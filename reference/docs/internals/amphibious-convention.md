# AmphibiousConvention: Naming and Indexing Internals

This page documents the `AmphibiousConvention` class (`farms_amphibious/model/convention.py`, 519 lines), which provides the naming and indexing scheme that connects oscillators, joints, links, and drives in the amphibious CPG model. Every maintainer working on `farms_amphibious` needs to understand this class because it is the single source of truth for mapping between array indices and human-readable names.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_amphibious/model/convention.py` | 519 | `AmphibiousConvention`, `BodyPartKind`, `Side` |
| `farms_core/options.py` | 53 | `Options` base class (dict subclass) |

## Call graph / entry points

```
AmphibiousOptions (YAML)
  └─ AmphibiousConvention.from_amphibious_options(animat_options)
       └─ AmphibiousConvention.from_morphology(morphology, **kwargs)
            └─ AmphibiousConvention.__init__(**kwargs)
```

The convention object is then passed to:

- `AmphibiousData.from_options(convention, ...)` — to pre-allocate state arrays with correct sizes
- `AmphibiousController(convention, ...)` — to map oscillator outputs to joint commands
- `DescendingDrive` subclasses — to know where brain/spine indices are in the drive array
- `ode_oscillators_sparse()` — indirectly, via the data containers that were sized by the convention

## Class hierarchy

```
Options (dict subclass)
  └─ AmphibiousConvention
```

`Options` (in `farms_core/options.py`) is a thin `dict` subclass that provides `__getattr__` and `__setattr__` delegating to `self[key]` / `self.__getitem__`. This means attribute access (`convention.n_joints_body`) and dict access (`convention['n_joints_body']`) are equivalent.

## Enums

### `BodyPartKind`

```python
class BodyPartKind(Enum):
    BRAIN = 0
    AXIAL = 1
    LIMB = 2
```

Used by `drvindex2information()` to classify a drive index as belonging to a brain drive, a body (axial) drive, or a leg (limb) drive.

### `Side`

```python
class Side(Enum):
    LEFT = 0
    RIGHT = 1
```

- `from_index(index)`: Asserts `0 <= index < 2`. Returns `LEFT` for 0, `RIGHT` for 1.
- `letter()`: Returns `"L"` or `"R"` (first letter, uppercased).
- `__repr__()`: Returns `"left"` or `"right"` (lowercase name).

## Constructor: `__init__(self, **kwargs)`

```python
def __init__(self, **kwargs):
    super().__init__()
    self.descending_drives_names = ['brain_left', 'brain_right']
    self.n_joints_body = kwargs.pop('n_joints_body')
    self.n_links_body = kwargs.pop('n_links_body', self.n_joints_body + 1)
    self.single_osc_body = kwargs.pop('single_osc_body', False)
    self.single_osc_legs = kwargs.pop('single_osc_legs', False)
    self.n_dof_legs = kwargs.pop('n_dof_legs')
    self.n_legs = kwargs.pop('n_legs')
    self.n_joints_passive = kwargs.pop('n_joints_passive')
    self.links_names = kwargs.pop('links_names', [...])  # generated if not provided
    self.joints_names = kwargs.pop('joints_names', [...])  # generated if not provided
    ...
    assert not kwargs, kwargs  # Reject unknown kwargs
```

### Parameters

| Parameter | Type | Required | Default | Description |
|---|---|---|---|---|
| `n_joints_body` | int | Yes | — | Number of body (spine) joints |
| `n_links_body` | int | No | `n_joints_body + 1` | Number of body links (one more than joints) |
| `single_osc_body` | bool | No | `False` | If `True`, one oscillator per body joint (no L/R pair) |
| `single_osc_legs` | bool | No | `False` | If `True`, one oscillator per leg joint (no L/R pair) |
| `n_dof_legs` | int | Yes | — | Degrees of freedom per leg |
| `n_legs` | int | Yes | — | Total number of legs (both sides). Must be even. |
| `n_joints_passive` | int | Yes | — | Number of passive joints (e.g., eyes) |
| `links_names` | list | No | Generated | Custom link names. If `links` kwarg is present, extracted from `link['name']`. Otherwise auto-generated. |
| `joints_names` | list | No | Generated | Custom joint names. Same fallback logic as `links_names`. |

**Critical assertion at line 78**: `len(self.joints_names) >= n_joints_all()`. If you provide fewer names than the total joint count, construction fails with a detailed error message listing the body/legs/passive breakdown.

**Critical assertion at line 86**: `assert not kwargs, kwargs`. Any unknown keyword argument causes construction failure. This is a strict constructor — typos in parameter names are caught immediately.

### Default joint name generation (lines 60–75)

When `joints_names` is not provided and there is no `joints` kwarg:

```python
joints_names = (
    [f'joint_body_{joint_i}' for joint_i in range(self.n_joints_body)]
    + [
        f'joint_leg_{leg_i}_{"R" if side_i else "L"}_{joint_i}'
        for leg_i in range(self.n_legs // 2)
        for side_i in range(2)
        for joint_i in range(self.n_dof_legs)
    ]
    + [f'joint_passive_{joint_i}' for joint_i in range(self.n_joints_passive)]
)
```

The iteration order for leg joints is: **leg pair first, then side (L then R), then DOF within the leg**. So for 2 leg pairs, 2 DOF:

```
joint_leg_0_L_0, joint_leg_0_L_1, joint_leg_0_R_0, joint_leg_0_R_1,
joint_leg_1_L_0, joint_leg_1_L_1, joint_leg_1_R_0, joint_leg_1_R_1
```

### Default link name generation (lines 46–58)

When `links_names` is not provided and there is no `links` kwarg:

```python
links_names = (
    [f'link_body_{link_i}' for link_i in range(self.n_joints_body + 1)]
    + [
        f'link_leg_{leg_i}_{"R" if side_i else "L"}_{joint_i}'
        for leg_i in range(self.n_legs // 2)
        for side_i in range(2)
        for joint_i in range(self.n_dof_legs)
    ]
)
```

Note: body links have `n_joints_body + 1` entries (one more than joints, because the spine has one extra link at the head). Leg links do NOT include a passive section.

## Construction classmethods

### `from_morphology(cls, morphology, **kwargs)`

```python
@classmethod
def from_morphology(cls, morphology, **kwargs):
    return cls(
        n_joints_body=morphology['n_joints_body'],
        n_joints_passive=morphology['n_joints_passive'],
        n_dof_legs=morphology['n_dof_legs'],
        n_legs=morphology['n_legs'],
        links_names=morphology.links_names(),
        joints_names=morphology.joints_names(),
        **kwargs,
    )
```

Takes a `MorphologyOptions` object (which is a dict-like `Options` with `links_names()` and `joints_names()` methods). Passes `single_osc_body` and `single_osc_legs` through kwargs.

### `from_amphibious_options(cls, animat_options)`

```python
@classmethod
def from_amphibious_options(cls, animat_options):
    network = animat_options.control.network
    return cls.from_morphology(
        morphology=animat_options.morphology,
        single_osc_body=network.single_osc_body,
        single_osc_legs=network.single_osc_legs,
    )
```

This is the standard entry point: given an `AmphibiousOptions` loaded from YAML, it pulls morphology parameters and the `single_osc_body`/`single_osc_legs` flags from the network options.

## Count methods

These methods compute sizes used throughout the framework. They are called frequently during data allocation and controller initialization.

| Method | Formula | Description |
|---|---|---|
| `n_joints_all()` | `n_joints_body + n_joints_legs() + n_joints_passive` | Total joints including passive |
| `n_joints_active()` | `n_joints_body + n_joints_legs()` | Active joints (no passive) |
| `n_joints_legs()` | `n_legs * n_dof_legs` | Total leg joints |
| `n_legs_pair()` | `n_legs // 2` | Number of leg pairs |
| `n_links_legs()` | `n_dof_legs * n_legs` | Total leg links |
| `n_opbj()` | `1 if single_osc_body else 2` | Oscillators per body joint |
| `n_oplj()` | `1 if single_osc_legs else 2` | Oscillators per leg joint |
| `n_osc_body()` | `n_opbj() * n_joints_body` | Total body oscillators |
| `n_osc_leg()` | `n_oplj() * n_dof_legs` | Oscillators per leg |
| `n_osc_legs()` | `n_oplj() * n_joints_legs()` | Total leg oscillators |
| `n_osc()` | `n_osc_body() + n_osc_legs()` | Total oscillators |
| `n_drives_brain()` | `len(descending_drives_names)` | Always 2 (`brain_left`, `brain_right`) |
| `n_drives()` | `n_drives_brain() + n_osc()` | Total drives |
| `n_states()` | `n_osc + n_osc + n_joints_active` | State vector size (phases + amplitudes + offsets) |

## Oscillator indexing

### Layout

```
[ body oscillators ] [ leg oscillators ]
```

Body oscillators are indexed first, then leg oscillators. Within body oscillators, each joint contributes `n_opbj()` oscillators (1 or 2). When `single_osc_body=False`, the two oscillators per joint are ordered **Left then Right** (side 0 = Left, side 1 = Right).

### `body_osc_indices(joint_i)`

```python
def body_osc_indices(self, joint_i):
    assert 0 <= joint_i < self.n_joints_body, (
        f'Joint must be < {self.n_joints_body}, got {joint_i}'
    )
    index = self.n_opbj() * joint_i
    return list(range(index, index + self.n_opbj()))
```

For `single_osc_body=False`: `body_osc_indices(3)` returns `[6, 7]` (joint 3, oscillators 6=L, 7=R).
For `single_osc_body=True`: `body_osc_indices(3)` returns `[3]` (just one oscillator).

### `bodyosc2index(joint_i, side=0)`

```python
def bodyosc2index(self, joint_i, side=0):
    if self.single_osc_body:
        assert side == 0, f'No oscillator side for joint {joint_i}'
    return self.body_osc_indices(joint_i)[side]
```

Returns the flat index for a specific oscillator within a body joint. If `single_osc_body=True`, passing `side=1` raises an `AssertionError`.

### `bodyosc2name(joint_i, side=0)`

```python
def bodyosc2name(self, joint_i, side=0):
    if self.single_osc_body:
        assert side == 0, f'No oscillator side for joint {joint_i}'
    return (
        f'osc_body_{joint_i}'
        if self.single_osc_body
        else f'osc_body_{joint_i}_{"R" if side else "L"}'
    )
```

Names follow the pattern `osc_body_{joint_i}_{L|R}`. When `single_osc_body=True`, the side suffix is omitted.

### `leg_osc_indices(**kwargs)`

This method has two calling conventions:

**Convention 1 — by flat joint index** (`index=` kwarg):

```python
def leg_osc_indices(self, **kwargs):
    leg_opj = self.n_oplj()
    if 'index' in kwargs:
        index = (
            self.n_osc_body()
            + leg_opj * (kwargs.pop('index') - self.n_joints_body)
        )
```

The `index` here is a **joint** index (not an oscillator index). It must be >= `n_joints_body`. The offset into the oscillator array is computed as `n_osc_body() + leg_opj * (joint_index - n_joints_body)`.

**Convention 2 — by leg/side/joint decomposition**:

```python
    else:
        leg_i, side_i, joint_i = (
            kwargs.pop(key) for key in ('leg_i', 'side_i', 'joint_i')
        )
        n_legs = self.n_legs
        n_legs_dof = self.n_dof_legs
        assert 0 <= leg_i < n_legs, f'Leg must be < {n_legs//2}, got {leg_i}'
        assert 0 <= side_i < 2, f'Body side must be < 2, got {side_i}'
        assert 0 <= joint_i < n_legs_dof, f'Joint must be < {n_legs_dof}, got {joint_i}'
        index = (
            self.n_osc_body()
            + leg_i * 2 * n_legs_dof * leg_opj  # 2 legs (L+R) in a pair
            + side_i * n_legs_dof * leg_opj     # offset within pair
            + joint_i * leg_opj                  # offset within leg
        )
```

**Important**: `leg_i` in the assertion checks `0 <= leg_i < n_legs` but the error message says `Leg must be < {n_legs//2}`. This is because `leg_i` is a **pair index** (0, 1, 2, ...) where each pair contains a Left and Right leg. The assertion uses `n_legs` (total) but the actual valid range is `0` to `n_legs//2 - 1`. The assertion is overly permissive — it would allow `leg_i = n_legs - 1` but the indexing math would produce an out-of-bounds oscillator index.

### `legosc2index(leg_i, side_i, joint_i, side=0)`

```python
def legosc2index(self, leg_i, side_i, joint_i, side=0):
    if self.single_osc_legs:
        assert side == 0, f'No oscillator side for legs ({leg_i}, {side_i}, {joint_i})'
    else:
        assert 0 <= side < 2, f'Oscillator side must be < 2, got {side}'
    return self.leg_osc_indices(
        leg_i=leg_i, side_i=side_i, joint_i=joint_i,
    )[side]
```

The `side` parameter here is the **oscillator side within the leg joint** (0 or 1), NOT the body side. When `single_osc_legs=False`, each leg joint has two oscillators (representing opposing muscles), and `side` selects between them.

### `legosc2name(leg_i, side_i, joint_i, side=0)`

```python
def legosc2name(self, leg_i, side_i, joint_i, side=0):
    return (
        f'osc_leg_{leg_i}_{"R" if side_i else "L"}_{joint_i}'
        if self.single_osc_legs
        else f'osc_leg_{leg_i}_{"R" if side_i else "L"}_{joint_i}_{side}'
    )
```

Name pattern: `osc_leg_{pair_index}_{L|R}_{dof_index}[_{osc_side}]`. The osc_side suffix is omitted when `single_osc_legs=True`.

### `osc_indices(joint_i)`

Dispatches to body or leg indexing based on the joint index:

```python
def osc_indices(self, joint_i):
    return (
        self.body_osc_indices(joint_i)
        if joint_i < self.n_joints_body
        else self.leg_osc_indices(index=joint_i)
    )
```

## Reverse lookup: index to information

### `oscindex2information(osc_i)`

Returns a dict decomposing a flat oscillator index. This is the inverse of all the indexing methods above.

```python
def oscindex2information(self, osc_i):
    information = {}
    n_oscillators = self.n_osc()
    n_body_oscillators = self.n_osc_body()
    assert 0 <= osc_i < n_oscillators, (...)
    information['body'] = osc_i < n_body_oscillators
    if information['body']:
        information['joint_i'] = osc_i if self.single_osc_body else osc_i // 2
        information['side'] = 0 if self.single_osc_body else (osc_i % 2)
    else:
        index_i = osc_i - n_body_oscillators
        information['side'] = 0 if self.single_osc_legs else (index_i % 2)
        n_osc_leg = self.n_oplj() * self.n_dof_legs
        n_osc_leg_pair = 2 * n_osc_leg
        information['leg'] = index_i // n_osc_leg
        information['leg_i'] = index_i // n_osc_leg_pair
        information['side_i'] = 0 if (index_i % n_osc_leg_pair) < n_osc_leg else 1
        information['joint_i'] = (index_i % n_osc_leg) // self.n_oplj()
    return information
```

**Walkthrough for body oscillators** (when `single_osc_body=False`):
- `joint_i = osc_i // 2`: oscillator 0,1 → joint 0; oscillator 2,3 → joint 1; etc.
- `side = osc_i % 2`: even indices are Left (0), odd indices are Right (1).

**Walkthrough for leg oscillators** (when `single_osc_legs=False`):
- `index_i = osc_i - n_body_oscillators`: local index within leg oscillator space
- `side = index_i % 2`: oscillator side within leg joint (0 or 1)
- `n_osc_leg = 2 * n_dof_legs`: oscillators for one leg (both sides of a single-leg pair member)
- `n_osc_leg_pair = 4 * n_dof_legs`: oscillators for one leg pair (L+R)
- `leg = index_i // n_osc_leg`: index within one leg pair member (0 or 1 for L/R)
- `leg_i = index_i // n_osc_leg_pair`: which leg pair (0, 1, 2, ...)
- `side_i`: 0 (Left) if the index within the pair is < n_osc_leg, else 1 (Right)
- `joint_i = (index_i % n_osc_leg) // 2`: which DOF within the leg

### `oscindex2name(index)` and `oscname2index(name)`

```python
def oscindex2name(self, index):
    parameters = self.oscindex2information(index)
    body = parameters.pop('body')
    if not body:
        parameters.pop('leg')
    return (
        self.bodyosc2name(**parameters) if body
        else self.legosc2name(**parameters)
    )

def oscname2index(self, name):
    osc_names = [self.oscindex2name(index) for index in range(self.n_osc())]
    return osc_names.index(name)
```

**Performance note**: `oscname2index` iterates over ALL oscillators to build a name list and then does `.index()`. For large animats this is O(n) per lookup. Do not call this in a hot loop.

## Joint indexing

### Layout

```
[ body joints ] [ leg joints ] [ passive joints ]
```

| Section | Start | End |
|---|---|---|
| Body joints | 0 | `n_joints_body - 1` |
| Leg joints | `n_joints_body` | `n_joints_active() - 1` |
| Passive joints | `n_joints_active()` | `n_joints_all() - 1` |

### `legjoint2index(leg_i, side_i, joint_i)`

```python
def legjoint2index(self, leg_i, side_i, joint_i):
    return (
        self.n_joints_body
        + leg_i * 2 * self.n_dof_legs
        + side_i * self.n_dof_legs
        + joint_i
    )
```

Note: the multiplier is `2 * n_dof_legs` (not `n_legs`), because within one leg pair there are two legs (L+R), each with `n_dof_legs` joints.

### `joint2legindices(joint_i)`

Reverse of `legjoint2index`:

```python
def joint2legindices(self, joint_i):
    assert self.n_joints_body <= joint_i < self.n_joints_active(), (...)
    j_i = joint_i - self.n_joints_body
    dof_i = j_i % self.n_dof_legs
    side_i = ((j_i - dof_i) // self.n_dof_legs) % 2
    leg_i = (j_i - side_i * self.n_dof_legs - dof_i) // (4 * self.n_dof_legs)
    return (leg_i, side_i, dof_i)
```

**Walkthrough** for `joint_i = 15`, `n_joints_body=11`, `n_dof_legs=2`:
1. `j_i = 15 - 11 = 4`
2. `dof_i = 4 % 2 = 0`
3. `side_i = ((4 - 0) // 2) % 2 = 2 % 2 = 0` (Left)
4. `leg_i = (4 - 0 * 2 - 0) // (4 * 2) = 4 // 8 = 0` (first pair)

So joint 15 → leg pair 0, Left side, DOF 0.

### `jointindex2information(joint_i)`

Returns a dict with `body` (bool), `joint_i`/`name` (for body joints) or `leg_i`/`side_i`/`joint_i`/`name` (for leg joints). Similar logic to `oscindex2information`.

## Link indexing

### `bodylink2name(link_i)` / `bodyjoint2name(link_i)`

```python
def bodylink2name(self, link_i):
    n_body = self.n_joints_body + 1
    assert 0 <= link_i < n_body, f'Body must be < {n_body}, got {link_i}'
    return self.links_names[link_i]
```

Both `bodylink2name` and `bodyjoint2name` simply index into `self.links_names` / `self.joints_names`. Despite the name `bodyjoint2name`, it takes a link index (0 to `n_joints_body`).

### `leglink2index(leg_i, side_i, joint_i)`

```python
def leglink2index(self, leg_i, side_i, joint_i):
    return (
        n_body_links
        + leg_i * 2 * n_legs_dof
        + side_i * n_legs_dof
        + joint_i
    )
```

Same formula as `legjoint2index` but offset by `n_links_body` instead of `n_joints_body`.

### `feet_links_names()`

```python
def feet_links_names(self):
    return [
        self.leglink2name(leg_i, side_i, self.n_dof_legs - 1)
        for leg_i in range(self.n_legs // 2)
        for side_i in range(2)
    ]
```

Returns the last link of each leg — the foot. Iterates over leg pairs then sides.

## Contact indexing

### `contactleglink2index(leg_i, side_i)`

```python
def contactleglink2index(self, leg_i, side_i):
    return 2 * leg_i + side_i
```

This is a **separate, simpler** indexing scheme for contact sensors. It does NOT correspond to the main link/joint index space. Contact sensors are indexed by leg pair and side only — there is one contact sensor per foot.

### `contactleglink2name(leg_i, side_i)`

```python
def contactleglink2name(self, leg_i, side_i):
    return self.leglink2name(leg_i, side_i, self.n_dof_legs - 1)
```

Returns the link name of the foot (last DOF link).

## Drive indexing

Drives are the descending signals that modulate oscillator frequencies and amplitudes. The drive array layout is:

```
[ brain drives ] [ body drives ] [ leg drives ]
```

### Brain drives

Always exactly 2: `['brain_left', 'brain_right']` (hardcoded at line 38). These are the highest-level descending drives controlled by `DescendingDrive` subclasses.

### Body and leg drives

Body drives have one entry per body oscillator (`n_osc_body()` entries). Leg drives have one entry per leg oscillator (`n_osc_legs()` entries). Total drives: `n_drives() = n_drives_brain() + n_osc() = 2 + n_osc()`.

### Drive naming

| Drive type | Naming pattern | Example |
|---|---|---|
| Brain | `drive_brain_{L\|R}` | `drive_brain_L` |
| Body | `drive_body_{joint_i}_{L\|R}` | `drive_body_3_R` |
| Leg | `drive_leg_{leg_i}_{L\|R}_{joint_i}_{L\|R}` | `drive_leg_0_L_1_R` |

For leg drives, the first `L|R` is the body side (`side_i`), and the second `L|R` is the oscillator side (`side`). When `single_osc_body=True`, brain drives still have L/R (always 2), but body drives use side=0 only.

### `drvindex2information(drv_i)`

```python
def drvindex2information(self, drv_i):
    information = {}
    n_drives = self.n_drives()
    assert 0 <= drv_i < n_drives, (...)
    n_drives_brain = self.n_drives_brain()
    n_drives_body = self.n_osc_body()
    information['body_part_kind'] = (
        BodyPartKind.BRAIN
        if drv_i < n_drives_brain
        else BodyPartKind.AXIAL
        if n_drives_brain <= drv_i < n_drives_brain + n_drives_body
        else BodyPartKind.LIMB
    )
```

Then for each body part kind, it extracts:

- **BRAIN**: `side = Side.from_index(0 if single_osc_body else (drv_i % 2))`
- **AXIAL**: `joint_i = drive_body_index if single_osc_body else drive_body_index // 2`, `side = Side.from_index(0 if single_osc_body else (drv_i % 2))`
- **LIMB**: Same decomposition as `oscindex2information` for legs

### `driveindex2name(index)`

```python
def driveindex2name(self, index):
    info = self.drvindex2information(index)
    body_part_kind = info.pop('body_part_kind')
    if body_part_kind is BodyPartKind.LIMB:
        info.pop('leg')
    return (
        self.drive_brain_name(**info) if body_part_kind is BodyPartKind.BRAIN
        else self.drive_body_name(**info) if body_part_kind is BodyPartKind.AXIAL
        else self.drive_leg_name(**info)
    )
```

## State array layout

The `n_states()` method returns the total state vector size:

```python
def n_states(self):
    n_osc = self.n_osc()
    n_joints = self.n_joints_active()
    return (
        n_osc            # Phases (one per oscillator)
        + n_osc          # Amplitudes (one per oscillator)
        + n_joints       # Joint offsets (one per active joint)
    )
```

This is the layout used by the ODE state vector. See [ODE Internals](ode-internals.md) for the mathematical equations computed over this state.

## Worked example: Salamander robot

Configuration: `n_joints_body=11`, `n_legs=4`, `n_dof_legs=2`, `n_joints_passive=0`, `single_osc_body=False`, `single_osc_legs=False`.

### Computed sizes

| Quantity | Calculation | Value |
|---|---|---|
| `n_joints_legs()` | 4 × 2 | 8 |
| `n_joints_active()` | 11 + 8 | 19 |
| `n_joints_all()` | 11 + 8 + 0 | 19 |
| `n_opbj()` | — | 2 |
| `n_oplj()` | — | 2 |
| `n_osc_body()` | 2 × 11 | 22 |
| `n_osc_legs()` | 2 × 8 | 16 |
| `n_osc()` | 22 + 16 | 38 |
| `n_drives_brain()` | — | 2 |
| `n_drives()` | 2 + 38 | 40 |
| `n_states()` | 38 + 38 + 19 | 95 |

### Oscillator index to name mapping

| Index | Name | Body/Leg | Joint | Side |
|---|---|---|---|---|
| 0 | `osc_body_0_L` | Body | 0 | L |
| 1 | `osc_body_0_R` | Body | 0 | R |
| 2 | `osc_body_1_L` | Body | 1 | L |
| 3 | `osc_body_1_R` | Body | 1 | R |
| ... | ... | ... | ... | ... |
| 20 | `osc_body_10_L` | Body | 10 | L |
| 21 | `osc_body_10_R` | Body | 10 | R |
| 22 | `osc_leg_0_L_0_0` | Leg | pair 0, L, DOF 0 | osc 0 |
| 23 | `osc_leg_0_L_0_1` | Leg | pair 0, L, DOF 0 | osc 1 |
| 24 | `osc_leg_0_L_1_0` | Leg | pair 0, L, DOF 1 | osc 0 |
| 25 | `osc_leg_0_L_1_1` | Leg | pair 0, L, DOF 1 | osc 1 |
| 26 | `osc_leg_0_R_0_0` | Leg | pair 0, R, DOF 0 | osc 0 |
| ... | ... | ... | ... | ... |
| 37 | `osc_leg_1_R_1_1` | Leg | pair 1, R, DOF 1 | osc 1 |

### Drive index to name mapping

| Index | Name | Kind |
|---|---|---|
| 0 | `drive_brain_L` | BRAIN |
| 1 | `drive_brain_R` | BRAIN |
| 2 | `drive_body_0_L` | AXIAL |
| 3 | `drive_body_0_R` | AXIAL |
| ... | ... | ... |
| 23 | `drive_body_10_R` | AXIAL |
| 24 | `drive_leg_0_L_0_L` | LIMB |
| ... | ... | ... |
| 39 | `drive_leg_1_R_1_R` | LIMB |

## How to integrate: adding a new body segment

To add a 12th body joint to the salamander:

1. Update the YAML `morphology.n_joints_body` from 11 to 12.
2. The convention will auto-generate `joint_body_11` and `link_body_12` in the default name lists.
3. Two new oscillators will be created: `osc_body_11_L` (index 22) and `osc_body_11_R` (index 23), pushing all leg oscillator indices up by 2.
4. The drive array gains `drive_body_11_L` and `drive_body_11_R`.
5. **You must also update**: the SDF model (new link + joint), the oscillator network YAML (new oscillator entries, phase biases, connectivity), and any drive/controller YAML.

**Do NOT** hardcode oscillator indices anywhere in your code. Always use `convention.bodyosc2index(joint_i, side)` or `convention.legosc2index(leg_i, side_i, joint_i, side)` to compute indices dynamically.

## How to integrate: adding a new leg pair

1. Update `morphology.n_legs` from 4 to 6 (always even).
2. The convention auto-generates joint names for the new pair: `joint_leg_2_L_0`, `joint_leg_2_L_1`, `joint_leg_2_R_0`, `joint_leg_2_R_1`.
3. New oscillators are appended after existing leg oscillators.
4. New drives are appended after existing leg drives.
5. **Update**: SDF model, network YAML (new leg oscillators with phase biases), drive/controller YAML.

## How to extend: subclassing AmphibiousConvention

```python
from farms_amphibious.model.convention import AmphibiousConvention

class MyCustomConvention(AmphibiousConvention):
    def __init__(self, **kwargs):
        # Pop custom parameters before calling super
        self.my_custom_param = kwargs.pop('my_custom_param', 'default')
        super().__init__(**kwargs)

    def n_custom_thing(self):
        """Override or add new counting methods"""
        return self.n_osc() + 5  # Example
```

**Important**: If you override `__init__`, always pop your custom kwargs before calling `super().__init__()`, because the parent constructor asserts `not kwargs`. If you add new parameters, they must be consumed before the parent sees them.

## Common failure modes

### 1. Unknown kwargs assertion (line 86)

```python
assert not kwargs, kwargs
```

Any typo in parameter names (e.g., `n_joint_body` instead of `n_joints_body`) will cause an `AssertionError` with the dict of remaining kwargs. The fix is to use the exact parameter names documented above.

### 2. Joint name count mismatch (line 78)

```python
assert len(self.joints_names) >= n_joints_all()
```

If you provide custom `joints_names` that are too short, construction fails. The error message includes the body/legs/passive breakdown to help debug.

### 3. Index out of range in indexing methods

Every indexing method (`body_osc_indices`, `leg_osc_indices`, `legosc2index`, etc.) has assertions that check `0 <= index < limit`. These will raise `AssertionError` with descriptive messages. Common cause: passing a leg pair index that is too large (e.g., `leg_i=3` when there are only 2 pairs).

### 4. `single_osc_body=True` but passing `side=1`

```python
if self.single_osc_body:
    assert side == 0, f'No oscillator side for joint {joint_i}'
```

If the convention was constructed with `single_osc_body=True`, there is only one oscillator per body joint. Passing `side=1` (or any non-zero value) to `bodyosc2index` or `bodyosc2name` raises an `AssertionError`.

### 5. `oscname2index` performance

`oscname2index` builds a full list of all oscillator names on every call. For large animats (38+ oscillators) called in a loop, this can be slow. Cache the result if you need repeated lookups:

```python
# Bad (O(n) per call):
for name in names:
    idx = convention.oscname2index(name)

# Good (O(n) once, then O(1) per lookup):
name_to_idx = {
    convention.oscindex2name(i): i
    for i in range(convention.n_osc())
}
for name in names:
    idx = name_to_idx[name]
```

## What NOT to assume

1. **`leg_i` is a pair index, not an individual leg index.** `leg_i=0` means the first leg pair (both left and right legs of that pair). The valid range is `0` to `n_legs//2 - 1`, not `0` to `n_legs - 1`.

2. **Left always comes before Right.** In all name generation and indexing, side 0 = Left, side 1 = Right. This is consistent across oscillators, joints, links, and drives.

3. **Passive joints have no oscillators.** They are included in `n_joints_all()` and `joints_names` but do NOT contribute to `n_osc()` or `n_states()`. The ODE state vector only contains active joints.

4. **`bodyjoint2name` takes a link index, not a joint index.** Despite its name, it indexes into `self.joints_names[link_i]` where `link_i` ranges from 0 to `n_joints_body` (inclusive). This is because the naming was designed so that joint `i` connects link `i` to link `i+1`.

5. **`n_links_body` defaults to `n_joints_body + 1`.** A spine with N joints has N+1 links (head + N segments). If you override this, you must ensure the SDF model matches.

6. **Brain drives are always 2.** The `descending_drives_names = ['brain_left', 'brain_right']` is hardcoded. Even with `single_osc_body=True`, you always get 2 brain drives. This is because brain drives represent left/right descending pathways, not oscillator counts.

7. **The `Side` enum uses `LEFT=0`, `RIGHT=1`.** This is the same convention as the integer `side` parameter in all indexing methods. Do not confuse `Side` (body side L/R) with the oscillator `side` parameter in `legosc2index` (which selects between the two opposing muscle oscillators within a leg joint).
