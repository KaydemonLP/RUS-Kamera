#pragma once

bool IsPixelWhite( int iColor );
bool IsPixelWhite( int red, int green, int blue );

struct Color{
    Color(){};
    Color( int red, int green, int blue ){ r = red; g = green; b = blue; }
    static Color FromInt(int iColor)
    {
        // Hex to colors
        int red = iColor & 0xFF0000;
        red = red >> (4 * 4);
        int green = iColor & 0x00FF00;
        green = green >> (4 * 2);
        int blue = iColor & 0x0000FF;

        return Color(red,green,blue);
    }

    int r;
    int g;
    int b;
};

bool IsPixelWhite( int iColor )
{
  Color color = Color::FromInt(iColor);

  return IsPixelWhite( color.r, color.g, color.b );
}

bool IsPixelWhite( int red, int green, int blue )
{
  float average = (float)(red + green + blue) / 3.0f;

  // Too dark
  if( average < 70 )
    return false;

  float red_deviation = abs(average - red);
  float green_deviation = abs(average - green);
  float blue_deviation = abs(average - blue);

  float max_deviation = std::max(red_deviation, std::max(green_deviation, blue_deviation));
  float average_deviation = (float)(red_deviation + green_deviation + blue_deviation) / 3.0f;

  
  if( average_deviation > 10 )
    return false;
  
  if( max_deviation > 15 )
    return false;
  
  return true;
}
