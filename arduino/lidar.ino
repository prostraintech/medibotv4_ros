void lidar (int state)
{
  if (lid_1 == 1 && lid_2 == 0 && lid_3 == 0)
  {
    motor(movement); // regular movement
  }
  else if (lid_1 == 1 && lid_2 == 1 && lid_3 == 0)
  {
    movement = 2;
    motor(movement);  // slow
  }
  else if (lid_1 == 1 && lid_2 == 1 && lid_3 == 1)
  {
    movement = 6;
    motor(movement); // stop
  }
}
