# Integration flow

Start intent -> compatibility adapter obtains config/clocks -> Run service creates generation/state -> adapter records/starts existing owners -> UI reads service snapshot.
Stop -> service freezes/retires generation -> UI admits next run -> adapter requests optional storage/teardown -> late completion checks original generation.
Detection pair -> immutable snapshot/request -> actor/worker -> immutable outcome -> record/journal -> detached workbook/UI projections.
Recovery click -> selected method/current worker identity -> coordinator command -> actor validates identity -> FAILED original result + replaces slot -> result routing/journal + UI observes new worker generation.
