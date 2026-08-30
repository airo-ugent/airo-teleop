from functools import wraps

from .errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


def check_if_not_connected(func):
    """Raise DeviceNotConnectedError if the device is not connected."""

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__} is not connected. Run `.connect()` first."
            )
        return func(self, *args, **kwargs)

    return wrapper


def check_if_already_connected(func):
    """Raise DeviceAlreadyConnectedError if the device is already connected."""

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                f"{self.__class__.__name__} is already connected."
            )
        return func(self, *args, **kwargs)

    return wrapper
