"""Canonical WIRIS Ethernet Stream SDK command catalog.

The catalog is based on Workswell's official ``WIRIS & GIS Ethernet Stream
SDK`` user manual, revision ``241106EN`` (6 November 2024, firmware 2.10.60
and higher).  It intentionally contains command variants as separate entries
so the Workstation preset selector can expose every documented choice while
still allowing the raw command editor to be used for a device-specific value.

Workstation only operates WIRIS Enterprise units, so commands the manual
documents as exclusive to a different product line -- WIRIS AGRO (CWSI/crop
stress, section 5.8), WIRIS Security (section 5.7), GIS 320 (section 5.11),
and AGRO's Colourmap (section 5.13) -- are deliberately left out rather than
offered as dead presets a WIRIS Enterprise camera cannot execute.
"""

from __future__ import annotations

from Workstation.workstation_core.wiris_zoom import LINE_TERMINATOR


WIRIS_SDK_MANUAL_URL = "https://my.workswell.eu/link/latest/1653482886wf2Pr9x8SKHQFZ9PN7f9"
WIRIS_SDK_MANUAL_REVISION = "241106EN"
WIRIS_SDK_MANUAL_FIRMWARE = "2.10.60+"


def _entry(category: str, label: str, command: str, description: str) -> tuple[str, str, str, str, str]:
    return (category, label, command, LINE_TERMINATOR, description)


WIRIS_COMMAND_CATALOG: tuple[tuple[str, str, str, str, str], ...] = (
    # 5.1 Activation
    _entry("Activation", "Check SDK activation", "IACT", "SDK 활성화 상태를 TRUE/FALSE로 읽습니다."),
    _entry("Activation", "Activate SDK", "ACTV <ACTIVATION-CODE>", "활성화 코드로 SDK를 활성화합니다. 전송 전에 실제 코드로 바꾸세요."),
    _entry("Activation", "Get Ethernet Mode", "GETH", "Ethernet/RTSP 모드 상태를 TRUE/FALSE로 읽습니다."),
    _entry("Activation", "Set Ethernet Mode ON", "SETH TRUE", "Ethernet/RTSP 모드를 켜서 영상 송출을 시작합니다."),
    _entry("Activation", "Set Ethernet Mode OFF", "SETH FALSE", "Ethernet/RTSP 모드를 끕니다."),
    _entry("Activation", "Set SDK Port", "SSDK <port>", "SDK 제어 TCP 포트를 변경합니다."),

    # 5.2 Basic commands
    _entry("Basic", "Check connection", "HIWS", "연결 확인 명령입니다. 정상 응답은 OK입니다."),
    _entry("Basic", "Set delimiter NULL", "SDLM NULL", "명령 구분자를 NULL 문자로 설정합니다."),
    _entry("Basic", "Set delimiter LINE", "SDLM LINE", "명령 구분자를 줄바꿈 문자로 설정합니다."),
    _entry("Basic", "Get delimiter", "GDLM", "현재 명령 구분자 설정을 읽습니다."),
    _entry("Basic", "Get serial number", "GSRN", "카메라 시리얼 번호를 읽습니다."),
    _entry("Basic", "Get article number", "GATN", "카메라 article 번호를 읽습니다."),
    _entry("Basic", "Get firmware version", "GFWV", "카메라 펌웨어 버전을 읽습니다."),
    _entry("Basic", "Get CPU temperature", "GTCU", "CPU 온도를 읽습니다."),
    _entry("Basic", "Get thermal core temperature", "GTIC", "열화상 core 온도를 읽습니다."),
    _entry("Basic", "Get internal temperature", "GTIN", "카메라 내부 전자부 온도를 읽습니다."),
    _entry("Basic", "Get fan power", "GFPW", "팬 출력 상태를 읽습니다."),

    # 5.3 Menu navigation / 5.4 GPS / 5.5 Units
    _entry("Menu", "Move up", "MOVE UP", "카메라 메뉴에서 위로 이동합니다."),
    _entry("Menu", "Move down", "MOVE DOWN", "카메라 메뉴에서 아래로 이동합니다."),
    _entry("Menu", "Move OK", "MOVE OK", "카메라 메뉴에서 현재 항목을 확인합니다."),
    _entry("Menu", "Move cancel", "MOVE CANCEL", "카메라 메뉴에서 취소하거나 뒤로 이동합니다."),
    _entry("GPS", "Get GPS coordinates", "GGPS", "카메라에 제공된 GPS 좌표를 읽습니다."),
    _entry("Units", "Set temperature units Kelvin", "STUT K", "온도 단위를 Kelvin으로 설정합니다."),
    _entry("Units", "Set temperature units Celsius", "STUT C", "온도 단위를 Celsius로 설정합니다."),
    _entry("Units", "Set temperature units Fahrenheit", "STUT F", "온도 단위를 Fahrenheit로 설정합니다."),
    _entry("Units", "Get temperature units", "GTUT", "현재 온도 단위(K/C/F)를 읽습니다."),

    # 5.6 WIRIS Pro / WIRIS Enterprise / GIS thermal parameters
    _entry("Thermal", "Set range mode automatic", "SRMD AUTOMATIC", "열화상 range mode를 AUTOMATIC으로 설정합니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Set range mode manual", "SRMD MANUAL", "열화상 range mode를 MANUAL로 설정합니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Set range mode span", "SRMD SPAN", "열화상 range mode를 SPAN으로 설정합니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Get range mode", "GRMD", "현재 열화상 range mode를 읽습니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Set manual range", "SRMM <min> <max>", "수동 열화상 range의 최소/최대 온도를 설정합니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Get manual range", "GRMM", "수동 열화상 range의 최소/최대 온도를 읽습니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Set span range", "SRWC <center> <window>", "SPAN range의 중심/창 값을 설정합니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Get span range", "GRWC", "SPAN range의 중심/창 값을 읽습니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Set environment range", "SREN <min> <max>", "환경 온도 range의 최소/최대 값을 설정합니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Get environment range", "GREN", "현재 환경 온도 range를 읽습니다. (WWP/WWE/GIS)"),
    _entry("Thermal", "Get environment range options", "GREL", "카메라가 지원하는 환경 온도 range 목록을 읽습니다. (WWP/WWE/GIS)"),

    # 5.9 Appearance
    _entry("Appearance", "Set layout inspection", "SLAY INSPECTION", "HDMI layout을 INSPECTION으로 설정합니다."),
    _entry("Appearance", "Set layout security", "SLAY SECURITY", "HDMI layout을 SECURITY로 설정합니다."),
    _entry("Appearance", "Set layout fullscreen", "SLAY FULLSCREEN", "HDMI layout을 FULLSCREEN으로 설정합니다."),
    _entry("Appearance", "Set layout picture-in-picture", "SLAY PIP", "HDMI layout을 PIP로 설정합니다."),
    _entry("Appearance", "Set picture-in-picture opacity", "STTY <opacity>", "PIP 열화상 overlay 투명도를 설정합니다(10-100%)."),
    _entry("Appearance", "Set main camera thermal", "SMCA THERMO", "열화상 카메라를 메인 카메라로 설정합니다."),
    _entry("Appearance", "Set main camera visible", "SMCA VISIBLE", "가시광 카메라를 메인 카메라로 설정합니다."),
    _entry("Appearance", "Get main camera", "GMCA", "현재 메인 카메라 설정을 읽습니다."),
    _entry("Appearance", "Set GPS info position", "SGPI POSITION", "화면 GPS 정보 표시를 위치 정보로 설정합니다."),
    _entry("Appearance", "Set GPS info altitude/speed", "SGPI ALTITUDE_SPEED", "화면 GPS 정보 표시를 고도/속도 정보로 설정합니다."),
    _entry("Appearance", "Get GPS info mode", "GGPI", "현재 GPS 정보 표시 모드를 읽습니다."),

    # 5.10 Zoom
    _entry("Zoom", "Zoom in", "SZIN", "선택한 카메라를 줌 인합니다."),
    _entry("Zoom", "Zoom out", "SZOT", "선택한 카메라를 줌 아웃합니다."),
    _entry("Zoom", "Set simultaneous zoom ON", "SZSM TRUE", "열화상/가시광 카메라를 동시에 줌하도록 설정합니다."),
    _entry("Zoom", "Set simultaneous zoom OFF", "SZSM FALSE", "열화상/가시광 동시 줌을 끕니다."),
    _entry("Zoom", "Get simultaneous zoom", "GZSM", "동시 줌 설정 상태를 읽습니다."),
    _entry("Zoom", "Set thermal zoom", "SZTN <index>", "열화상 줌 index를 설정합니다."),
    _entry("Zoom", "Get thermal zoom", "GZTV", "현재 열화상 줌 index/ratio를 읽습니다."),
    _entry("Zoom", "Get thermal zoom list", "GZTL", "지원되는 열화상 줌 index/ratio 목록을 읽습니다."),
    _entry("Zoom", "Set visible zoom", "SZVN <index>", "가시광 줌 index를 설정합니다."),
    _entry("Zoom", "Get visible zoom", "GZVV", "현재 가시광 줌 index/ratio를 읽습니다."),
    _entry("Zoom", "Get visible zoom list", "GZVL", "지원되는 가시광 줌 index/ratio 목록을 읽습니다."),

    # 5.12 Palettes
    _entry("Palette", "Get current palette", "GPTE", "현재 thermal palette index/name을 읽습니다. (WIRIS AGRO 제외)"),
    _entry("Palette", "Set palette by name", "SPTE <name>", "thermal palette를 이름으로 설정합니다. (WIRIS AGRO 제외)"),
    _entry("Palette", "Set palette by index", "SPTI <index>", "thermal palette를 index로 설정합니다. (WIRIS AGRO 제외)"),
    _entry("Palette", "Get palette list", "GPTL", "지원되는 thermal palette 목록을 읽습니다. (WIRIS AGRO 제외)"),
    _entry("Palette", "Get palette lookup table", "GPLV", "현재 palette의 256-entry RGB lookup table을 읽습니다. (WIRIS AGRO 제외)"),

    # 5.14 Capture and record
    _entry("Capture/Record", "Capture image", "CPTR", "카메라 자체 이미지 캡처를 트리거합니다."),
    _entry("Capture/Record", "Is capturing", "ICPT", "카메라 캡처 진행 상태를 읽습니다."),
    _entry("Capture/Record", "Recording start", "RCRS", "카메라 자체 녹화를 시작합니다."),
    _entry("Capture/Record", "Recording finish", "RCRF", "카메라 자체 녹화를 종료합니다."),
    _entry("Capture/Record", "Is recording", "IRCR", "카메라 녹화 상태를 읽습니다."),
    _entry("Capture/Record", "Set periodic capture", "SPIC <period>", "주기 캡처 주기를 설정합니다(0-60초, 0은 OFF)."),
    _entry("Capture/Record", "Get periodic capture", "GPIC", "현재 주기 캡처 설정을 읽습니다."),
    _entry("Capture/Record", "Get geofencing trigger", "GGFT", "geofencing trigger 상태를 읽습니다."),
    _entry("Capture/Record", "Geofencing trigger ON", "SGFT ON", "geofencing trigger를 켭니다."),
    _entry("Capture/Record", "Geofencing trigger OFF", "SGFT OFF", "geofencing trigger를 끕니다."),
    _entry("Capture/Record", "Set geofencing maximum altitude", "SBFL <max_altitude_m>", "geofencing 최대 고도를 설정합니다(40-400m)."),
    _entry("Capture/Record", "Get geofencing maximum altitude", "GBFL", "geofencing 최대 고도를 읽습니다."),
    _entry("Capture/Record", "Set geofencing minimum altitude", "SEFL <min_altitude_m>", "geofencing 최소 고도를 설정합니다(0-400m)."),
    _entry("Capture/Record", "Get geofencing minimum altitude", "GEFL", "geofencing 최소 고도를 읽습니다."),
    _entry("Capture/Record", "Set geofencing maximum speed", "SCSB <max_speed_mps>", "geofencing 최대 캡처 속도를 설정합니다(0.1-10m/s)."),
    _entry("Capture/Record", "Get geofencing maximum speed", "GCSB", "geofencing 최대 캡처 속도를 읽습니다."),

    # 5.15 Temperature values
    _entry("Temperature", "Set alarm mode above", "SALM ABOVE", "온도 alarm mode를 ABOVE로 설정합니다."),
    _entry("Temperature", "Set alarm mode below", "SALM BELOW", "온도 alarm mode를 BELOW로 설정합니다."),
    _entry("Temperature", "Set alarm mode between", "SALM BETWEEN", "온도 alarm mode를 BETWEEN으로 설정합니다."),
    _entry("Temperature", "Set alarm mode outside", "SALM OUTSIDE", "온도 alarm mode를 OUTSIDE로 설정합니다."),
    _entry("Temperature", "Set alarm mode off", "SALM OFF", "온도 alarm을 끕니다."),
    _entry("Temperature", "Get alarm mode", "GALM", "현재 온도 alarm mode를 읽습니다."),
    _entry("Temperature", "Set alarm values", "SALV <above> <below>", "온도 alarm의 위/아래 임계값을 설정합니다."),
    _entry("Temperature", "Get alarm values", "GALV", "온도 alarm의 위/아래 임계값을 읽습니다."),
    _entry("Temperature", "Set alarm colours", "SALC <below> <between> <above>", "온도 alarm 색상을 아래/사이/위 순서로 설정합니다."),
    _entry("Temperature", "Get alarm colours", "GALC", "현재 온도 alarm 색상을 읽습니다."),
    _entry("Temperature", "Get thermal extremes", "GTEX", "thermal camera의 현재 MIN/MAX/CENTER 값을 읽습니다."),
    _entry("Temperature", "Get ROI temperature", "GROT <xstart> <ystart> <xend> <yend>", "사각 ROI의 온도 통계를 읽습니다(최대 3Hz)."),
    _entry("Temperature", "Get pixel temperature", "GOPT <column> <line>", "지정한 한 픽셀의 온도를 읽습니다(최대 3Hz)."),

    # 5.16 Update
    _entry("Update", "Search for update", "CUPD", "카메라 저장장치에서 update 파일을 검색합니다. 완료까지 오래 걸릴 수 있습니다."),
    _entry("Update", "Install update", "IUPD", "검색된 update를 설치하고 카메라를 재부팅합니다."),

    # 5.17 Thermal camera parameters
    _entry("Thermal parameters", "Set emissivity", "STEM <emissivity>", "열화상 emissivity를 설정합니다(WIRIS Security 제외)."),
    _entry("Thermal parameters", "Get emissivity", "GTEM", "현재 열화상 emissivity를 읽습니다."),
    _entry("Thermal parameters", "Set reflected temperature", "STRT <temperature>", "반사 온도를 설정합니다(WIRIS Security 제외)."),
    _entry("Thermal parameters", "Get reflected temperature", "GTRT", "반사 온도를 읽습니다."),
    _entry("Thermal parameters", "Set atmospheric temperature", "STAT <temperature>", "대기 온도를 설정합니다(WIRIS Security 제외)."),
    _entry("Thermal parameters", "Get atmospheric temperature", "GTAT", "대기 온도를 읽습니다."),
    _entry("Thermal parameters", "Set image interpolation ON", "STII TRUE", "열화상 image interpolation을 켭니다."),
    _entry("Thermal parameters", "Set image interpolation OFF", "STII FALSE", "열화상 image interpolation을 끕니다."),
    _entry("Thermal parameters", "Get image interpolation", "GTII", "열화상 image interpolation 상태를 읽습니다."),
    _entry("Thermal parameters", "Set synchronous shutter ON", "STSC TRUE", "synchronous shutter 설정을 켭니다."),
    _entry("Thermal parameters", "Set synchronous shutter OFF", "STSC FALSE", "synchronous shutter 설정을 끕니다."),
    _entry("Thermal parameters", "Get synchronous shutter", "GTSC", "synchronous shutter 설정 상태를 읽습니다."),
    _entry("Thermal parameters", "Set shutter period", "STCP <seconds>", "synchronous shutter 주기를 설정합니다(120-1800초)."),
    _entry("Thermal parameters", "Get shutter period", "GTCP", "현재 camera shutter 주기를 읽습니다."),
    _entry("Thermal parameters", "Get shutter elapsed time", "GTSL", "마지막 shutter 이후 경과 시간을 읽습니다."),
    _entry("Thermal parameters", "Get next shutter time", "GTSN", "다음 synchronous shutter까지 남은 시간을 읽습니다."),
    _entry("Thermal parameters", "Perform thermal shutter", "IMCR", "열화상 camera shutter를 즉시 실행합니다."),
    _entry("Thermal parameters", "Set current thermal lens", "SLEN <index>", "현재 thermal lens index를 설정합니다."),
    _entry("Thermal parameters", "Get current thermal lens", "GLEN", "현재 thermal lens index를 읽습니다."),
    _entry("Thermal parameters", "Get thermal lens list", "GLEL", "사용 가능한 thermal lens 목록을 읽습니다."),

    # 5.18 Image and video settings
    _entry("Image/Video", "Set radiometric JPEG ON", "SIRJ TRUE", "radiometric JPEG 저장을 켭니다."),
    _entry("Image/Video", "Set radiometric JPEG OFF", "SIRJ FALSE", "radiometric JPEG 저장을 끕니다."),
    _entry("Image/Video", "Get radiometric JPEG", "GIRJ", "radiometric JPEG 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set radiometric TIFF ON", "SIRT TRUE", "radiometric TIFF 저장을 켭니다."),
    _entry("Image/Video", "Set radiometric TIFF OFF", "SIRT FALSE", "radiometric TIFF 저장을 끕니다."),
    _entry("Image/Video", "Get radiometric TIFF", "GIRT", "radiometric TIFF 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set CWSI TIFF ON", "SICT TRUE", "CWSI TIFF 저장을 켭니다."),
    _entry("Image/Video", "Set CWSI TIFF OFF", "SICT FALSE", "CWSI TIFF 저장을 끕니다."),
    _entry("Image/Video", "Get CWSI TIFF", "GICT", "CWSI TIFF 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set super-resolution ON", "SISR TRUE", "super-resolution 저장을 켭니다."),
    _entry("Image/Video", "Set super-resolution OFF", "SISR FALSE", "super-resolution 저장을 끕니다."),
    _entry("Image/Video", "Get super-resolution", "GISR", "super-resolution 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set screenshot ON", "SISS TRUE", "screenshot 저장을 켭니다."),
    _entry("Image/Video", "Set screenshot OFF", "SISS FALSE", "screenshot 저장을 끕니다."),
    _entry("Image/Video", "Get screenshot", "GISS", "screenshot 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set visible image ON", "SIVI TRUE", "visible image 저장을 켭니다."),
    _entry("Image/Video", "Set visible image OFF", "SIVI FALSE", "visible image 저장을 끕니다."),
    _entry("Image/Video", "Get visible image", "GIVI", "visible image 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set visible HR image ON", "SIHR TRUE", "visible high-resolution image 저장을 켭니다."),
    _entry("Image/Video", "Set visible HR image OFF", "SIHR FALSE", "visible high-resolution image 저장을 끕니다."),
    _entry("Image/Video", "Get visible HR image", "GIHR", "visible high-resolution image 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set radiometric video ON", "SVTH TRUE", "radiometric video 저장을 켭니다."),
    _entry("Image/Video", "Set radiometric video OFF", "SVTH FALSE", "radiometric video 저장을 끕니다."),
    _entry("Image/Video", "Get radiometric video", "GVTH", "radiometric video 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set thermal encoded video ON", "SVTE TRUE", "thermal encoded video 저장을 켭니다."),
    _entry("Image/Video", "Set thermal encoded video OFF", "SVTE FALSE", "thermal encoded video 저장을 끕니다."),
    _entry("Image/Video", "Get thermal encoded video", "GVTE", "thermal encoded video 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set visible video ON", "SVVI TRUE", "visible video 저장을 켭니다."),
    _entry("Image/Video", "Set visible video OFF", "SVVI FALSE", "visible video 저장을 끕니다."),
    _entry("Image/Video", "Get visible video", "GVVI", "visible video 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set visible HR video ON", "SVHR TRUE", "visible high-resolution video 저장을 켭니다."),
    _entry("Image/Video", "Set visible HR video OFF", "SVHR FALSE", "visible high-resolution video 저장을 끕니다."),
    _entry("Image/Video", "Get visible HR video", "GVHR", "visible high-resolution video 저장 상태를 읽습니다."),
    _entry("Image/Video", "Set image location SSD", "SILC SSD", "이미지 저장 위치를 SSD로 설정합니다."),
    _entry("Image/Video", "Set image location SD card", "SILC SD_CARD", "이미지 저장 위치를 SD card로 설정합니다."),
    _entry("Image/Video", "Set image location flash drive", "SILC FLASH_DRIVE", "이미지 저장 위치를 flash drive로 설정합니다."),
    _entry("Image/Video", "Get image location", "GILC", "현재 이미지 저장 위치를 읽습니다."),
    _entry("Image/Video", "Set HR camera resolution 4656x3496", "SHRR 4656x3496", "WIRIS Enterprise HR camera 해상도를 설정하고 재부팅합니다."),
    _entry("Image/Video", "Set HR camera resolution 2320x1744", "SHRR 2320x1744", "WIRIS Enterprise HR camera 해상도를 설정하고 재부팅합니다."),
    _entry("Image/Video", "Set HR camera resolution 1920x1080", "SHRR 1920x1080", "WIRIS Enterprise HR camera 해상도를 설정하고 재부팅합니다."),
    _entry("Image/Video", "Get HR camera resolution", "GHRR", "WIRIS Enterprise HR camera 해상도/FPS를 읽습니다."),

    # 5.19 Laser rangefinder
    _entry("LRF", "Show laser position ON", "SLRF TRUE", "LRF 위치 표시를 HDMI에 켭니다. (WIRIS Enterprise 일부 모델)"),
    _entry("LRF", "Show laser position OFF", "SLRF FALSE", "LRF 위치 표시를 HDMI에서 끕니다. (WIRIS Enterprise 일부 모델)"),
    _entry("LRF", "Get laser position display", "GLRF", "LRF HDMI 위치 표시 상태를 읽습니다."),
    _entry("LRF", "Get laser distance", "GLRD", "LRF(Laser Range Finder)가 측정한 거리 값을 읽습니다."),

    # 5.20 Date and time
    _entry("Date/Time", "Get date/time", "GDTI", "현재 카메라 날짜/시간을 읽습니다."),
    _entry("Date/Time", "Set Unix time", "STUS <unix_seconds> <microseconds>", "카메라 시간을 Unix epoch 초+마이크로초로 설정합니다. 재부팅 후 유지되지 않을 수 있습니다."),
    _entry("Date/Time", "Set date/time text", "SDTI yyyy/MM/dd-hh:mm:ss", "카메라 시간을 날짜 문자열로 설정합니다. 재부팅 후 유지되지 않을 수 있습니다."),

    # 5.21 Memory
    _entry("Memory", "Get memory status", "GMST", "SSD/SD_CARD/FLASH_DRIVE 상태를 읽습니다."),
    _entry("Memory", "Get memory size", "GMSI", "SSD/SD_CARD/FLASH_DRIVE 용량을 읽습니다."),
    _entry("Memory", "Get memory free percent", "GMFR", "SSD/SD_CARD/FLASH_DRIVE 여유 공간 비율을 읽습니다."),
    _entry("Memory", "Get captured images", "GMCP", "메모리별 캡처 이미지 수를 읽습니다."),
    _entry("Memory", "Get recorded thermal video", "GTRC", "저장된 radiometric thermal video 길이를 초 단위로 읽습니다."),
    _entry("Memory", "Get recorded visible video", "GVRC", "저장된 visible video 길이를 초 단위로 읽습니다."),
    _entry("Memory", "Get estimated saving time", "GEST", "현재 예상 저장 시간을 초 단위로 읽습니다."),
    _entry("Memory", "Copy data to SD card", "CPST SD_CARD", "SSD의 데이터를 SD card로 복사하기 시작합니다."),
    _entry("Memory", "Copy data to flash drive", "CPST FLASH_DRIVE", "SSD의 데이터를 flash drive로 복사하기 시작합니다."),
    _entry("Memory", "Get copy progress", "CPSS", "데이터 복사 상태/진행률을 읽습니다."),

    # 5.22 Stream
    _entry("Stream", "Get thermal resolution", "GTRE", "열화상 스트림 해상도를 읽습니다."),
    _entry("Stream", "Set thermal encoder bitrate", "SEBR THERMO <bitrate>", "열화상 인코더 bitrate를 설정합니다."),
    _entry("Stream", "Set visible encoder bitrate", "SEBR VISIBLE <bitrate>", "가시광 인코더 bitrate를 설정합니다."),
    _entry("Stream", "Get thermal encoder bitrate", "GEBR THERMO", "열화상 인코더 bitrate를 읽습니다."),
    _entry("Stream", "Get visible encoder bitrate", "GEBR VISIBLE", "가시광 인코더 bitrate를 읽습니다."),
    _entry("Stream", "Set thermal GOP size", "SEGS THERMO <gop>", "열화상 인코더 GOP 크기를 설정합니다."),
    _entry("Stream", "Set visible GOP size", "SEGS VISIBLE <gop>", "가시광 인코더 GOP 크기를 설정합니다."),
    _entry("Stream", "Get thermal GOP size", "GEGS THERMO", "열화상 인코더 GOP 크기를 읽습니다."),
    _entry("Stream", "Get visible GOP size", "GEGS VISIBLE", "가시광 인코더 GOP 크기를 읽습니다."),
    _entry("Stream", "Set thermal IDR interval", "SEIR THERMO <idr>", "열화상 인코더 IDR 간격을 설정합니다."),
    _entry("Stream", "Set visible IDR interval", "SEIR VISIBLE <idr>", "가시광 인코더 IDR 간격을 설정합니다."),
    _entry("Stream", "Get thermal IDR interval", "GEIR THERMO", "열화상 인코더 IDR 간격을 읽습니다."),
    _entry("Stream", "Get visible IDR interval", "GEIR VISIBLE", "가시광 인코더 IDR 간격을 읽습니다."),
    _entry("Stream", "Set thermal quantization quality", "SEQZ THERMO <qp>", "열화상 인코더 QP 값을 설정합니다(0-51)."),
    _entry("Stream", "Set visible quantization quality", "SEQZ VISIBLE <qp>", "가시광 인코더 QP 값을 설정합니다(0-51)."),
    _entry("Stream", "Get thermal quantization quality", "GEQZ THERMO", "열화상 인코더 QP 설정을 읽습니다."),
    _entry("Stream", "Get visible quantization quality", "GEQZ VISIBLE", "가시광 인코더 QP 설정을 읽습니다."),

    # 5.23 Trigger
    _entry("Trigger", "Set trigger correction", "STRG CORRECTION", "외부 trigger 입력을 보정 동작으로 설정합니다."),
    _entry("Trigger", "Set trigger capture", "STRG CAPTURE", "외부 trigger 입력을 이미지 캡처 동작으로 설정합니다."),
    _entry("Trigger", "Set trigger record", "STRG RECORD", "외부 trigger 입력을 녹화 제어 동작으로 설정합니다."),
    _entry("Trigger", "Disable trigger action", "STRG NA", "외부 trigger 동작을 사용하지 않도록 설정합니다."),
    _entry("Trigger", "Get trigger action", "GTRG", "현재 외부 trigger 동작 설정을 읽습니다."),

    # 5.24 System
    _entry("System", "Set IP address", "STIP <IPv4>", "카메라 IP 주소를 변경합니다. 연결이 끊길 수 있습니다."),
    _entry("System", "Set subnet mask", "STMS <mask>", "카메라 서브넷 마스크를 변경합니다."),
    _entry("System", "Get subnet mask", "GTMS", "현재 서브넷 마스크를 읽습니다."),
    _entry("System", "Set gateway", "STGW <gateway>", "카메라 gateway 주소를 변경합니다."),
    _entry("System", "Get gateway", "GTGW", "현재 gateway 주소를 읽습니다."),
    _entry("System", "Manual gateway ON", "SMGW TRUE", "수동 gateway 설정을 사용합니다."),
    _entry("System", "Manual gateway OFF", "SMGW FALSE", "수동 gateway 설정을 끕니다."),
    _entry("System", "Get manual gateway", "GMGW", "수동 gateway 사용 여부를 읽습니다."),
    _entry("System", "Restore default settings", "SDST", "카메라 기본 설정을 복원합니다. Ethernet mode/IP가 바뀔 수 있습니다."),
    _entry("System", "Shut down", "SHDW", "카메라를 종료합니다."),
    _entry("System", "Reboot", "REBT", "카메라를 재부팅합니다."),

    # Workstation-only raw entry; it is deliberately not an SDK command.
    _entry("Custom", "Custom SDK command", "", "SDK 명령을 직접 입력합니다."),
)


# Commands that can change device state, interrupt recording/streaming, or
# make the current control connection unavailable.  Normal zoom and Ethernet
# enable actions remain available without an extra confirmation because they
# are part of the existing connection workflow.
WIRIS_DANGEROUS_COMMAND_PREFIXES = (
    "ACTV",
    "SSDK",
    "SETH FALSE",
    "SDLM",
    "STUT",
    "SRMD",
    "SRMM",
    "SRWC",
    "SREN",
    "SLAY",
    "STTY",
    "SMCA",
    "SGPI",
    "SZSM",
    "SZTN",
    "SZVN",
    "SPTE",
    "SPTI",
    "CPTR",
    "RCRS",
    "RCRF",
    "SPIC",
    "SGFT",
    "SBFL",
    "SEFL",
    "SCSB",
    "SALM",
    "SALV",
    "SALC",
    "CUPD",
    "IUPD",
    "STEM",
    "STRT",
    "STAT",
    "STII",
    "STSC",
    "STCP",
    "IMCR",
    "SLEN",
    "SIRJ",
    "SIRT",
    "SICT",
    "SISR",
    "SISS",
    "SIVI",
    "SIHR",
    "SVTH",
    "SVTE",
    "SVVI",
    "SVHR",
    "SILC",
    "SHRR",
    "SLRF",
    "STUS",
    "SDTI",
    "CPST",
    "SEBR",
    "SEGS",
    "SEIR",
    "SEQZ",
    "STRG",
    "STIP",
    "STMS",
    "STGW",
    "SMGW",
    "SDST",
    "SHDW",
    "REBT",
)
