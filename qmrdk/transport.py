"""USBTMC transport to the QM-RDK over PyVISA (docs/protocol.md §1, §2, §3.7):
discovery, write/query, the SCPI error policy, frame flush and reopen after
re-enumeration. The only module that opens USB resources."""

import contextlib
import dataclasses
import time

from pyvisa import constants, errors, rname

VID = 0x2012
MAKER, MODEL = "Quonset Microwave", "QM4004"
CHUNK = 31
MAX_SAMPLES = 4096
NOT_READY = "Not Ready"
USB_QUERY = "USB?*::INSTR"
POWER_ON = -500
QUEUE_DEPTH = 10
TIMEOUT_MS = 2000
REOPEN_S = 10.0
POLL_S = 0.25
DOCUMENTED = (
    "SWEEP:FREQSTAR",
    "SWEEP:FREQSTOP",
    "SWEEP:RAMPTIME",
    "SWEEP:TYPE",
    "SWEEP:START",
    "SWEEP:STOP",
    "POWE:RF",
    "FREQ:LOCK",
    "FREQ:REF:DIV",
    "CAPT:FRAM",
    "SYST:IDEN",
    "SYST:SERNUM",
    "SYST:MODNUM",
    "SYST:FIRM",
    "SYST:VERS",
    "SYST:TEMP",
    "SYST:STAT",
    "SYST:ERR",
    "SYST:PRES",
    "SYST:REST",
    "SYST:CLRM",
    "*IDN",
    "*RST",
    "*SAV",
    "*RCL",
    "*CLS",
    "*ESE",
    "*ESR",
    "*OPC",
    "*OPT",
    "*SRE",
    "*STB",
    "*TRG",
    "*TST",
    "*WAI",
)


class DeviceError(RuntimeError):
    """The board is absent, reported an error or returned a malformed frame."""


class ScpiError(DeviceError):
    """A non-zero `SYST:ERR?` entry after `cmd`; `queue` holds every entry drained."""

    def __init__(self, code: int, text: str, cmd: str = "", queue=()):
        self.code, self.text, self.cmd = code, text, cmd
        self.queue = list(queue) or [(code, text)]
        super().__init__(f"{cmd}: {code},{text!r}" if cmd else f"{code},{text!r}")


class DeviceTimeout(DeviceError):
    """A read did not complete within the timeout."""


class Disconnected(DeviceError):
    """The USB session was lost (board reset, re-enumerated or unplugged)."""


class ForbiddenCommand(DeviceError):
    """A command outside the documented set the host uses (protocol §3)."""


@dataclasses.dataclass(frozen=True)
class Idn:
    """`*IDN?` fields."""

    maker: str
    model: str
    serial: str
    firmware: str

    @classmethod
    def parse(cls, resp: str) -> "Idn":
        fields = [f.strip() for f in resp.strip().split(",")]
        if len(fields) != 4 or fields[:2] != [MAKER, MODEL]:
            raise DeviceError(f"not a QM-RDK: {resp.strip()!r}")
        return cls(*fields)


def _manager():
    import pyvisa  # pylint: disable=import-outside-toplevel

    return pyvisa.ResourceManager("@py")


def parse_entry(resp: str) -> tuple[int, str]:
    """`<code>, "<text>"` of `SYST:ERR?` / `SYST:STAT?`."""
    code, _, text = resp.partition(",")
    try:
        return int(code), text.strip().strip('"')
    except ValueError as err:
        raise DeviceError(f"malformed status entry {resp!r}") from err


def same_serial(a: str, b: str) -> bool:
    """Serial numbers equal up to zero padding."""
    return a.lstrip("0") == str(b).lstrip("0")


def usb_serial(name: str) -> str | None:
    """USB serial string of a resource under the QM-RDK vendor ID, else None."""
    try:
        res = rname.parse_resource_name(name)
    except rname.InvalidResourceName:
        return None
    if res.interface_type_const != constants.InterfaceType.usb:
        return None
    vid = res.manufacturer_id
    if int(vid, 16 if vid.lower().startswith("0x") else 10) != VID:
        return None
    return res.serial_number


def resources(manager=None, serial: str | None = None) -> list[str]:
    """Resource names under the vendor ID, optionally of one serial number."""
    out = []
    manager = _manager() if manager is None else manager
    for name in manager.list_resources(USB_QUERY):
        found = usb_serial(name)
        if found is not None and (serial is None or same_serial(found, serial)):
            out.append(name)
    return out


def forbidden(message: str) -> str | None:
    """The first command of `message` outside the documented set the host
    uses (protocol §3), else None. Long forms and a leading colon are matched
    through their short-form prefixes."""
    for cmd in message.split(";"):
        head = cmd.strip().split(maxsplit=1)[0:1]
        if not head:
            continue
        nodes = head[0].upper().lstrip(":").rstrip("?").split(":")
        if not any(
            len(nodes) == len(short) and all(map(str.startswith, nodes, short))
            for short in (d.split(":") for d in DOCUMENTED)
        ):
            return cmd.strip()
    return None


def _io(fn, *args):
    """Call into the VISA resource, mapping its failures to DeviceError types."""
    try:
        return fn(*args)
    except errors.VisaIOError as err:
        if err.error_code == constants.StatusCode.error_timeout:
            raise DeviceTimeout(str(err)) from err
        raise Disconnected(str(err)) from err
    except (errors.Error, OSError) as err:
        raise Disconnected(str(err)) from err


class Transport:
    """One QM-RDK session. `manager` is a PyVISA resource manager (default the
    pyvisa-py backend); the board is `resource`, else the first one found,
    optionally of serial number `serial`."""

    def __init__(
        self,
        resource: str | None = None,
        serial: str | None = None,
        manager=None,
        timeout: int = TIMEOUT_MS,
    ):
        self.manager = _manager() if manager is None else manager
        self.timeout = timeout
        self.reopen_s = REOPEN_S
        self.res = None
        self.name, self.idn = None, None
        self._connect(resource, serial)

    def _connect(self, resource, serial):
        names = [resource] if resource else resources(self.manager, serial)
        failures = []
        for name in names:
            try:
                res = _io(self.manager.open_resource, name)
            except DeviceError as err:
                failures.append(f"{name}: {err}")
                continue
            try:
                res.read_termination = res.write_termination = "\n"
                res.timeout = self.timeout
                idn = Idn.parse(_io(res.query, "*IDN?"))
                if serial is not None and not same_serial(idn.serial, serial):
                    raise DeviceError(f"serial {idn.serial}, not {serial}")
            except DeviceError as err:
                with contextlib.suppress(errors.Error, OSError):
                    res.close()
                failures.append(f"{name}: {err}")
                continue
            self.res, self.name, self.idn = res, name, idn
            return
        which = "" if serial is None else f" with serial {serial}"
        raise DeviceError(
            f"no QM-RDK found{which}" + "".join(f"; {f}" for f in failures)
        )

    def _call(self, method: str, cmd: str):
        bad = forbidden(cmd)
        if bad:
            raise ForbiddenCommand(f"refusing undocumented {bad!r} (protocol §3)")
        if self.res is None:
            raise Disconnected(f"{cmd}: not connected")
        return _io(getattr(self.res, method), cmd)

    def write(self, cmd: str) -> None:
        """Send a program message without checking the error queue."""
        self._call("write", cmd)

    def query(self, cmd: str, timeout: int | None = None) -> str:
        """Send a query and read its response; `timeout` (ms) for this call only."""
        if timeout is None or self.res is None:
            return self._call("query", cmd).strip()
        old, self.res.timeout = self.res.timeout, timeout
        try:
            return self._call("query", cmd).strip()
        finally:
            with contextlib.suppress(errors.Error, OSError):
                self.res.timeout = old

    def errors(self, power_on: bool = False) -> list[tuple[int, str]]:
        """Drain the error queue; `-500 Power on` is consumed silently unless
        `power_on`."""
        out = []
        for _ in range(QUEUE_DEPTH + 2):
            code, text = parse_entry(self.query("SYST:ERR?"))
            if code == 0:
                return out
            if code != POWER_ON or power_on:
                out.append((code, text))
        raise DeviceError("error queue does not drain")

    def check(self, cmd: str = "") -> None:
        """Raise ScpiError for the first queued error (protocol §3.7)."""
        queue = self.errors()
        if queue:
            raise ScpiError(*queue[0], cmd, queue)

    def command(self, cmd: str) -> None:
        """Send a setter, then apply the error policy."""
        self.write(cmd)
        self.check(cmd)

    def flush(self) -> None:
        """Discard pending frame data (pyvisa-py has no USBTMC device clear)."""
        for _ in range(-(-MAX_SAMPLES // CHUNK) + 2):
            try:
                if self.query("CAPT:FRAM?") == NOT_READY:
                    return
            except DeviceTimeout:
                pass
        raise DeviceError("frame data does not drain")

    def wait_gone(self, deadline_s: float | None = None) -> None:
        """Poll until the session drops (after `*RST`), or the deadline passes."""
        end = time.monotonic() + (self.reopen_s if deadline_s is None else deadline_s)
        while time.monotonic() < end:
            try:
                self.query("*IDN?", timeout=int(POLL_S * 1e3))
            except DeviceError:
                return
            time.sleep(POLL_S)

    def reopen(self, deadline_s: float | None = None) -> None:
        """Rediscover the same board by serial number after re-enumeration."""
        self.close()
        end = time.monotonic() + (self.reopen_s if deadline_s is None else deadline_s)
        while True:
            try:
                self._connect(None, self.idn.serial)
                return
            except DeviceError as err:
                if time.monotonic() >= end:
                    raise DeviceError(
                        f"board {self.idn.serial} did not re-enumerate: {err}"
                    ) from err
            time.sleep(POLL_S)

    def close(self) -> None:
        """Release the resource; safe on a lost session."""
        if self.res is not None:
            with contextlib.suppress(errors.Error, OSError):
                self.res.close()
            self.res = None
