"""Scenario: reproducible vehicle batches shared by every controller.

One scenario is an immutable batch of vehicles with stable identities, so every
ceiling solve and every controller acts on the same physical instance.
Generation uses independent random streams for initial states, arrivals, and
movement allocation, so changing one factor leaves the others untouched across
matched cases.
"""
