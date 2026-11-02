from moveit_msgs.msg import AllowedCollisionEntry


def set_allowed_collision_pairs(matrix, pairs):
    names = list(matrix.entry_names)
    rows = [list(entry.enabled) for entry in matrix.entry_values]
    if len(rows) != len(names) or any(len(row) != len(names) for row in rows):
        raise ValueError("MoveIt returned an inconsistent allowed-collision matrix")

    for first, second, allowed in pairs:
        for name in (first, second):
            if name in names:
                continue
            names.append(name)
            for row in rows:
                row.append(False)
            rows.append([False] * len(names))

        first_index = names.index(first)
        second_index = names.index(second)
        rows[first_index][second_index] = allowed
        rows[second_index][first_index] = allowed

    matrix.entry_names = names
    matrix.entry_values = [
        AllowedCollisionEntry(enabled=row) for row in rows
    ]
    return matrix
