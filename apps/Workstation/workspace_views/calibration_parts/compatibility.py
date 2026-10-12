"""Finite legacy attributes; new components never consult the workspace facade."""


class ViewAlias:
    def __init__(self, component, attribute):
        self.component = component
        self.attribute = attribute

    def __get__(self, instance, owner=None):
        if instance is None:
            return self
        return getattr(instance._calibration_component(self.component), self.attribute)

    def __set__(self, instance, value):
        setattr(instance._calibration_component(self.component), self.attribute, value)
