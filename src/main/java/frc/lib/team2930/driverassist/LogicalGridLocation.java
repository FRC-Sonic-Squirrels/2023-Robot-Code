package frc.lib.team2930.driverassist;

public enum LogicalGridLocation {
  LOGICAL_BAY_1(PhysicalGridLocation.BLUE_PHYSICAL_BAY_1, PhysicalGridLocation.RED_PHYSICAL_BAY_1),

  LOGICAL_BAY_2(PhysicalGridLocation.BLUE_PHYSICAL_BAY_2, PhysicalGridLocation.RED_PHYSICAL_BAY_2),

  LOGICAL_BAY_3(PhysicalGridLocation.BLUE_PHYSICAL_BAY_3, PhysicalGridLocation.RED_PHYSICAL_BAY_3),

  LOGICAL_BAY_4(PhysicalGridLocation.BLUE_PHYSICAL_BAY_4, PhysicalGridLocation.RED_PHYSICAL_BAY_4),

  LOGICAL_BAY_5(PhysicalGridLocation.BLUE_PHYSICAL_BAY_5, PhysicalGridLocation.RED_PHYSICAL_BAY_5),

  LOGICAL_BAY_6(PhysicalGridLocation.BLUE_PHYSICAL_BAY_6, PhysicalGridLocation.RED_PHYSICAL_BAY_6),

  LOGICAL_BAY_7(PhysicalGridLocation.BLUE_PHYSICAL_BAY_7, PhysicalGridLocation.RED_PHYSICAL_BAY_7),

  LOGICAL_BAY_8(PhysicalGridLocation.BLUE_PHYSICAL_BAY_8, PhysicalGridLocation.RED_PHYSICAL_BAY_8),

  LOGICAL_BAY_9(PhysicalGridLocation.BLUE_PHYSICAL_BAY_9, PhysicalGridLocation.RED_PHYSICAL_BAY_9);

  public final PhysicalGridLocation bluePhysical;
  public final PhysicalGridLocation redPhysical;

  private LogicalGridLocation(PhysicalGridLocation bluePhysical, PhysicalGridLocation redPhysical) {
    this.bluePhysical = bluePhysical;
    this.redPhysical = redPhysical;
  }
}
