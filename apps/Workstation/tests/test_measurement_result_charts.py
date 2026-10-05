from dataclasses import replace
import re
import zipfile
from xml.etree import ElementTree as ET

from Workstation.tests.test_measurement_workbook import _populated_record
from Workstation.workstation_core.measurement_report import RESULT_CHART_FIELDS
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.measurement_workbook import write_measurement_workbook
from Workstation.workstation_core.measurement_xlsx_read import read_workbook

NS = {'c':'http://schemas.openxmlformats.org/drawingml/2006/chart',
      'a':'http://schemas.openxmlformats.org/drawingml/2006/main'}
FIELDS = ('forward_m','right_m','down_m','range_m','target_level_forward_m',
          'target_level_right_m','target_level_down_m','target_level_range_m',
          'target_latitude_deg','target_longitude_deg','target_altitude_ellipsoid_m','target_altitude_msl_m')


def chart_xml(path):
    with zipfile.ZipFile(path) as z:
        names = sorted((n for n in z.namelist() if re.fullmatch(r'xl/charts/chart\d+\.xml', n)),
                       key=lambda n:int(re.search(r'chart(\d+)\.xml', n).group(1)))
        return [ET.fromstring(z.read(name)) for name in names]


def plotted_values(series, sheets):
    ref = series.find('c:yVal/c:numRef/c:f', NS).text
    match = re.fullmatch(r"'?([^']+)'?!\$B\$(\d+)(?::\$B\$(\d+))?", ref)
    assert match, ref
    name, first, last = match.groups()
    last = last or first
    return [sheets[name][i][1] if i < len(sheets[name]) and len(sheets[name][i])>1 else None
            for i in range(int(first)-1,int(last))]


def test_twelve_charts_plot_correct_fields_and_method_styles(tmp_path):
    _, record = _populated_record(tmp_path)
    snapshot = record.snapshot()
    source = next(r for r in snapshot.method_results if r.status=='VALID')
    rows=[]
    for method_index, method in enumerate(MEASUREMENT_METHOD_IDS[:2]):
        for i,pair in enumerate(snapshot.pairs):
            # Deliberately distinct values expose field mixing and omissions.
            values={name: (metric+1)*10 + method_index + i*.125 for metric,name in enumerate(FIELDS)}
            if method_index==0:
                values['target_latitude_deg']=None
            rows.append(replace(source,pair_id=pair.pair_id,pair_sequence=pair.pair_sequence,method_id=method,
                                status='BLOCKED' if i==1 else 'ESTIMATED',derived={**source.derived,**values}))
    path=write_measurement_workbook(replace(snapshot,method_results=tuple(rows)),record.workbook_path)
    sheets=read_workbook(path)
    charts=chart_xml(path)
    assert tuple(field for field,_,_ in RESULT_CHART_FIELDS)==FIELDS
    assert len(charts)==12
    assert list(sheets)[5]=='05_그래프'
    for index,chart in enumerate(charts):
        series=chart.findall('.//c:scatterChart/c:ser',NS)
        expected_methods=[1] if index==8 else [0,1]
        assert len(series)==len(expected_methods)
        for curve, method_index in zip(series,expected_methods):
            values=plotted_values(curve,sheets)
            assert values==[(index+1)*10+method_index,None,(index+1)*10+method_index+.25]
        blanks = chart.find('.//c:dispBlanksAs',NS)
        assert blanks is None or blanks.attrib['val']=='gap'  # default OOXML mode
        axes = {axis.find('c:axPos', NS).attrib['val']: axis
                for axis in chart.findall('.//c:valAx', NS)}
        # Negative Down/altitude values must not move time labels into the plot.
        assert axes['b'].find('c:tickLblPos', NS).attrib['val'] == 'low'
        assert axes['b'].find('c:crosses', NS).attrib['val'] == 'min'
        visible_only = chart.find('c:chart/c:plotVisOnly', NS)
        assert visible_only is None or visible_only.attrib['val'] == '0'
    # Yaw90 is the same colour even when the preceding method lacks latitude.
    ordinary=charts[0].findall('.//c:scatterChart/c:ser',NS)[1]
    latitude=charts[8].findall('.//c:scatterChart/c:ser',NS)[0]
    assert ordinary.find('.//a:srgbClr',NS).attrib==latitude.find('.//a:srgbClr',NS).attrib
    assert charts[8].findall('.//c:valAx/c:numFmt',NS)[1].attrib['formatCode']=='0.000000000'


def test_empty_or_missing_fields_have_twelve_empty_slots_without_zero(tmp_path):
    _,record=_populated_record(tmp_path)
    path=write_measurement_workbook(replace(record.snapshot(),method_results=()),record.workbook_path)
    sheets=read_workbook(path)
    charts=chart_xml(path)
    assert len(charts)==12
    for chart in charts:
        assert '데이터 없음' in ''.join(chart.find('c:chart/c:title',NS).itertext())
        series=chart.find('.//c:scatterChart/c:ser',NS)
        assert plotted_values(series,sheets)==[None]
        for axis in chart.findall('.//c:valAx', NS):
            assert axis.find('c:delete', NS).attrib['val'] == '1'


def test_single_result_is_visible_as_a_marker_and_does_not_change_values(tmp_path):
    _,record=_populated_record(tmp_path)
    snapshot=record.snapshot()
    result=next(r for r in snapshot.method_results if r.status=='VALID')
    original=dict(result.derived)
    path=write_measurement_workbook(replace(snapshot,method_results=(result,)),record.workbook_path)
    charts=chart_xml(path)
    series=charts[3].find('.//c:scatterChart/c:ser',NS)
    assert series.find('c:marker/c:symbol',NS).attrib['val']=='circle'
    assert plotted_values(series,read_workbook(path))==[original['range_m']]
    assert result.derived==original
