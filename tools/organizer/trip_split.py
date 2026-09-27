"""Split real recordings by whole rides.

Neighbouring samples of one ride are not separate trials. A random cut of
rows would put almost the same instant in two splits. The closed test rides
are not opened for scoring.
"""

from __future__ import annotations


class RowLeak(ValueError):
    """One ride was placed in two splits."""


class ClosedTest(PermissionError):
    """The test rides stay closed."""


def assign_trips(ride_ids: list[str]) -> dict[str, list[str]]:
    """Assign whole ride ids. Sorted id order, never a shuffle of rows.

    One ride stays in train. Two rides become train and validation. The test
    list is non-empty only when a ride can be held out whole.
    """
    rides = sorted(set(ride_ids))
    if not rides:
        return {"train": [], "validation": [], "test": []}
    if len(rides) == 1:
        return {"train": rides, "validation": [], "test": []}
    if len(rides) == 2:
        return {"train": [rides[0]], "validation": [rides[1]], "test": []}
    n_test = max(1, len(rides) // 5)
    n_val = max(1, len(rides) // 5)
    n_train = len(rides) - n_val - n_test
    if n_train < 1:
        n_train = 1
        n_val = len(rides) - n_train - n_test
    return {
        "train": rides[:n_train],
        "validation": rides[n_train:n_train + n_val],
        "test": rides[n_train + n_val:],
    }


def bind_rows(rows: list[dict], assignment: dict[str, list[str]]) -> dict[str, list[dict]]:
    """Move every row of a ride into that ride's split."""
    owner: dict[str, str] = {}
    for name, rides in assignment.items():
        for ride in rides:
            if ride in owner:
                raise RowLeak(ride)
            owner[ride] = name
    out = {name: [] for name in ("train", "validation", "test")}
    for row in rows:
        ride = row["ride_id"]
        if ride not in owner:
            raise KeyError(ride)
        out[owner[ride]].append(row)
    return out


def assert_no_row_leak(rows: list[dict], groups: dict[str, list[int]]) -> None:
    """Reject an index split that cuts one ride across groups."""
    seen: dict[str, str] = {}
    for name, idxs in groups.items():
        for i in idxs:
            ride = rows[i]["ride_id"]
            prev = seen.get(ride)
            if prev is not None and prev != name:
                raise RowLeak(ride)
            seen[ride] = name


def open_for_score(name: str, assignment: dict[str, list[str]]) -> list[str]:
    """Return train or validation ride ids. Test stays closed."""
    if name == "test":
        raise ClosedTest("closed test rides are not scored")
    if name not in assignment:
        raise KeyError(name)
    return list(assignment[name])
