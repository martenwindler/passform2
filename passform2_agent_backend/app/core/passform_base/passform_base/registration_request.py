from dataclasses import InitVar, dataclass, field

from passform_msgs.srv import RegisterModule
import passform_util.network as nw

@dataclass
class RegistrationRequest():
    request: InitVar[RegisterModule.Request | None] = None
    bay_id: int | None = field(default=None)
    is_virtual: bool = field(init=False, default=False)

    def __post_init__(self, request):
        if self.bay_id is not None:
            return
        req_type = request.id_type.lower()
        if req_type == 'virtual':
            self.is_virtual = True
            return
        if req_type == 'fixed':
            self.bay_id = int(request.id)
        elif req_type == 'ip':
            self.bay_id = nw.get_bay_by_ip(request.id)
        else:
            raise KeyError(f'Unknown ID type "{request.id_type}".')
        assert self.bay_id > 0, 'bay_id is not allowed to be <= 0'

if __name__ == '__main__':
    print(RegistrationRequest(bay_id=3))
    print(RegistrationRequest(RegisterModule.Request(id_type='virtual')))
