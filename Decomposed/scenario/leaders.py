"""Same-lane predecessor used by headways and approach following."""


def leader_of(vehicle, vehicles):
    """Immediate leader in the same approach lane, or None for the head."""
    if vehicle.order == 0:
        return None
    for other in vehicles:
        if (other.app, other.lane, other.order) == (vehicle.app, vehicle.lane, vehicle.order - 1):
            return other
    raise ValueError(f"Vehicle {vehicle.vid} has no leader at order {vehicle.order - 1}")
