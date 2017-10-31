
void paintsharp() {

  int xres = 168;
  int yres = 144;
  int rc = 50;
  float alpha;

  //scale vario to angle
  alpha = PI * vario / (2 * 3.6);

  // clear screen
  sharp.fillRect(0, 0, xres, yres, BLACK);

  //draw edge
  projectborder(rc, xres, yres, alpha, 0);
  if (alpha > PI / 2) y = 0;
  if (alpha < -PI / 2) y = yres;
  sharp.fillRect(0, yres / 2, max(x, 20), y - yres / 2, WHITE);
  sharp.fillRoundRect(5, 5, xres - 10, yres - 10, rc - 10, BLACK);

  paintnumbers(rc);

  // cirle indicator
  projectborder(rc, xres, yres, -alpha / 2 - 0.3, 5);
  sharp.fillCircle(x, y, 8, WHITE);
  sharp.fillCircle(x, y, 4, BLACK);

  //triangle
  float triangle = 0.30; //angle of triangle needle in rad

  projectborder(rc, xres, yres, alpha, 0);
  float x1 = x + 20 * (cos(alpha - triangle));
  float y1 = y + 20 * (sin(alpha - triangle));
  float x2 = x + 20 * (cos(alpha + triangle));
  float y2 = y + 20 * (sin(alpha + triangle));
  sharp.fillTriangle(x, y, x1, y1, x2, y2, WHITE);

  sharp.refresh();
}


void paintnumbers(int rc) {

  int ndigits = floor(log10(abs(int(alt)))) + 1;
  int digitwidth = 22;

  // top numbers
  sharp.setTextSize(1);
  sharp.setTextColor(WHITE, BLACK);
  sharp.setFont(&Lato_Regular_40);
  sharp.setCursor(110 - ndigits * digitwidth, 58);
  sharp.print(int(alt));
  sharp.setFont(&DejaVu_Sans_Bold_15);
  sharp.print("m");

  // central small numbers
  sharp.setFont(&DejaVu_Sans_Bold_15);
  sharp.setCursor(105, 113);
  sharp.print("m/s");
  sharp.setCursor(110, 92);
  sharp.print("5s");

  // central number
  sharp.setCursor(24, 113);
  sharp.setTextColor(WHITE);
  sharp.setFont(&Lato_Regular_40);

  // STF indicator
  int counter = 0;
  int xstart, ystart;
  sharp.fillCircle(158, 72, 8, WHITE);
  if (avg >= -0.0001) {
    sharp.print("+");
    ystart = 55;
    while (ystart > 0 && counter <= avg - 1 && counter < 4) {
      sharp.fillTriangle(148, ystart, 168, ystart, 158, ystart - 10, WHITE);
      ystart -= 14;
      counter++;
    }
  } else {
    sharp.print("-");
    ystart = 89;
    while (ystart < 168 && counter <= abs(avg) - 1 && counter < 4) {
      sharp.fillTriangle(148, ystart, 168, ystart, 158, ystart + 10, WHITE);
      ystart += 14;
      counter++;
    }
  }
  sharp.setCursor(45, 113);

  if (abs(avg) < 10) sharp.println(abs(avg), 1);
  else sharp.println(abs(avg), 0);
}


