/*
===============================================================================
 Name        : DrawLine.c
 Author      : $RJ
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include <cr_section_macros.h>
#include <NXP/crp.h>
#include "LPC17xx.h"                        /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>



/* Be careful with the port number and location number, because

some of the location may not exist in that port. */

#define PORT_NUM            0


uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];


#define ST7735_TFTWIDTH 127
#define ST7735_TFTHEIGHT 159

#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_SLPOUT 0x11
#define ST7735_DISPON 0x29



#define swap(x, y) {x = x + y; y = x - y; x = x - y ;}

// defining color values

#define LIGHTBLUE 0x00FFE0
#define GREEN 0x00FF00
#define DARKBLUE 0x000033
#define BLACK 0x000000
#define BLUE 0x0007FF
#define RED 0xFF0000
#define MAGENTA 0x00F81F
#define WHITE 0xFFFFFF
#define PURPLE 0xCC33FF

#define UpperBD 19
#define PI      3.1415926

int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;


void spiwrite(uint8_t c)

{

 int pnum = 0;

 src_addr[0] = c;

 SSP_SSELToggle( pnum, 0 );

 SSPSend( pnum, (uint8_t *)src_addr, 1 );

 SSP_SSELToggle( pnum, 1 );

}



void writecommand(uint8_t c)

{

 LPC_GPIO0->FIOCLR |= (0x1<<21);

 spiwrite(c);

}



void writedata(uint8_t c)

{

 LPC_GPIO0->FIOSET |= (0x1<<21);

 spiwrite(c);

}



void writeword(uint16_t c)

{

 uint8_t d;

 d = c >> 8;

 writedata(d);

 d = c & 0xFF;

 writedata(d);

}



void write888(uint32_t color, uint32_t repeat)

{

 uint8_t red, green, blue;

 int i;

 red = (color >> 16);

 green = (color >> 8) & 0xFF;

 blue = color & 0xFF;

 for (i = 0; i< repeat; i++) {

  writedata(red);

  writedata(green);

  writedata(blue);

 }

}



void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)

{

 writecommand(ST7735_CASET);

 writeword(x0);

 writeword(x1);

 writecommand(ST7735_RASET);

 writeword(y0);

 writeword(y1);

}


void fillrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

 int16_t i;

 int16_t width, height;

 width = x1-x0+1;

 height = y1-y0+1;

 setAddrWindow(x0,y0,x1,y1);

 writecommand(ST7735_RAMWR);

 write888(color,width*height);

}



void lcddelay(int ms)

{

 int count = 24000;

 int i;

 for ( i = count*ms; i--; i > 0);

}



void lcd_init()

{

 int i;
 printf("LCD Demo Begins!!!\n");
 // Set pins P0.16, P0.21, P0.22 as output
 LPC_GPIO0->FIODIR |= (0x1<<16);

 LPC_GPIO0->FIODIR |= (0x1<<21);

 LPC_GPIO0->FIODIR |= (0x1<<22);

 // Hardware Reset Sequence
 LPC_GPIO0->FIOSET |= (0x1<<22);
 lcddelay(500);

 LPC_GPIO0->FIOCLR |= (0x1<<22);
 lcddelay(500);

 LPC_GPIO0->FIOSET |= (0x1<<22);
 lcddelay(500);

 // initialize buffers
 for ( i = 0; i < SSP_BUFSIZE; i++ )
 {

   src_addr[i] = 0;
   dest_addr[i] = 0;
 }

 // Take LCD display out of sleep mode
 writecommand(ST7735_SLPOUT);
 lcddelay(200);

 // Turn LCD display on
 writecommand(ST7735_DISPON);
 lcddelay(200);

}




void drawPixel(int16_t x, int16_t y, uint32_t color)

{

 if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))

 return;

 setAddrWindow(x, y, x + 1, y + 1);

 writecommand(ST7735_RAMWR);

 write888(color, 1);

}



/*****************************************************************************


** Descriptions:        Draw line function

**

** parameters:           Starting point (x0,y0), Ending point(x1,y1) and color

** Returned value:        None

**

*****************************************************************************/


void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

 int16_t slope = abs(y1 - y0) > abs(x1 - x0);

 if (slope) {

  swap(x0, y0);

  swap(x1, y1);

 }

 if (x0 > x1) {

  swap(x0, x1);

  swap(y0, y1);

 }

 int16_t dx, dy;

 dx = x1 - x0;

 dy = abs(y1 - y0);

 int16_t err = dx / 2;

 int16_t ystep;

 if (y0 < y1) {

  ystep = 1;

 }

 else {

  ystep = -1;

 }

 for (; x0 <= x1; x0++) {

  if (slope) {

   drawPixel(y0, x0, color);

  }

  else {

   drawPixel(x0, y0, color);

  }

  err -= dy;

  if (err < 0) {

   y0 += ystep;

   err += dx;

  }

 }

}


/*

 Main Function main()

*/
typedef struct {
	float X[UpperBD];
	float Y[UpperBD];
	float Z[UpperBD];
} pworld;

typedef struct {
	float X[UpperBD];
	float Y[UpperBD];
	float Z[UpperBD];
} pviewer;

typedef struct {
	float X[UpperBD];
	float Y[UpperBD];
} pperspective;

typedef struct {
	float X;
	float Y;
} perspective;

typedef struct {
    float r[UpperBD], g[UpperBD], b[UpperBD];
} pt_diffuse;

typedef struct {
	float Rho;
	float sPheta;
	float cPheta;
	float sPhi;
	float cPhi;
	float D_focal;
} viewerAngles;

viewerAngles world2ViewerAngles(float Xe, float Ye, float Ze, float D_focal)
{
	viewerAngles viewerAngles;
	viewerAngles.Rho = sqrt(pow(Xe,2) + pow(Ye,2) + pow(Ze,2));
	viewerAngles.sPheta = Ye / sqrt(pow(Xe,2) + pow(Ye,2));
	viewerAngles.cPheta = Xe / sqrt(pow(Xe,2) + pow(Ye,2));
	viewerAngles.sPhi = sqrt(pow(Xe,2) + pow(Ye,2)) / viewerAngles.Rho;
	viewerAngles.cPhi = Ze / viewerAngles.Rho;
	viewerAngles.D_focal = D_focal;
	return viewerAngles;
}

perspective transformPipeline(viewerAngles viewerAngles, int X, int Y, int Z)
{
	int viewerX = -viewerAngles.sPheta * X + viewerAngles.cPheta * Y;
	int viewerY = -viewerAngles.cPheta * viewerAngles.cPhi * X - viewerAngles.cPhi * viewerAngles.sPheta * Y + viewerAngles.sPhi * Z;
	int viewerZ = -viewerAngles.sPhi * viewerAngles.cPheta * X - viewerAngles.sPhi * viewerAngles.cPheta * Y -viewerAngles.cPheta * Z + viewerAngles.Rho;

	perspective perspective;
	perspective.X = viewerAngles.D_focal * viewerX / viewerZ;
	perspective.Y = viewerAngles.D_focal * viewerY / viewerZ;

	return perspective;
}

void mydisplay(float Xe, float Ye, float Ze, float D_focal)
{
	int Num_index = UpperBD - 1;
	float Rho = sqrt(pow(Xe,2) + pow(Ye,2) + pow(Ze,2));

	pworld  world;
	pviewer viewer;
	pperspective perspective;

	//define the x-y-z world coordinate
	world.X[0] = 0.0;    world.Y[0] =  0.0;   world.Z[0] =  0.0;    // origin
	world.X[1] = 50.0;   world.Y[1] =  0.0;   world.Z[1] =  0.0;    // x-axis
	world.X[2] = 0.0;    world.Y[2] =  50.0;  world.Z[2] =  0.0;    // y-axis
	world.X[3] = 0.0;    world.Y[3] =  0.0;   world.Z[3] =  50.0;   // y-axis

	//define box vertex
/*
	world.X[4] = 0.0;    world.Y[4] =  0.0;   world.Z[4] =  10.0;    //
	world.X[5] = 40.0;   world.Y[5] =  0.0;   world.Z[5] =  10.0;    //
	world.X[6] = 0.0;    world.Y[6] =  40.0;  world.Z[6] =  10.0;    //
	world.X[7] = 0.0;    world.Y[7] =  0.0;   world.Z[7] =  50.0;   //

	world.X[8] = 40.0;    world.Y[8] =  0.0;   world.Z[8] =  50.0;    //
	world.X[9] = 0.0;     world.Y[9] =  40.0;  world.Z[9] =  50.0;    //
	world.X[10] = 40.0;   world.Y[10] =  40.0;  world.Z[10] =  10.0;    //
	world.X[11] = 40.0;   world.Y[11] =  40.0;  world.Z[11] =  50.0;   //
*/
	world.X[4] = 0.0;    world.Y[4] =  0.0;   world.Z[4] =  10.0;    //
	world.X[5] = 40.0;   world.Y[5] =  0.0;   world.Z[5] =  10.0;    //
	world.X[6] = 40.0;   world.Y[6] =  40.0;  world.Z[6] =  10.0;    //
	world.X[7] = 0.0;    world.Y[7] =  40.0;  world.Z[7] =  10.0;    //

	world.X[8] = 0.0;    world.Y[8] =  0.0;   world.Z[8] =  50.0;   //
	world.X[9] = 40.0;    world.Y[9] =  0.0;   world.Z[9] =  50.0;    //
	world.X[10] = 40.0;   world.Y[10] =  40.0;  world.Z[10] =  50.0;   //
	world.X[11] = 0.0;     world.Y[11] =  40.0;  world.Z[11] =  50.0;    //


	float sPheta = Ye / sqrt(pow(Xe,2) + pow(Ye,2));
	float cPheta = Xe / sqrt(pow(Xe,2) + pow(Ye,2));
	float sPhi = sqrt(pow(Xe,2) + pow(Ye,2)) / Rho;
	float cPhi = Ze / Rho;

    //14 = normal vector, 13 = A, 12 = Ps, 8 = top left box vertex
    world.X[12] = -50; world.Y[12] = 20; world.Z[12] = 100; // Ps (point source)
    world.X[13] = 0; world.Y[13] = 0; world.Z[13] = 0; // arbitrary vector A on x-y plane
    world.X[14] = 0; world.Y[14] = 0; world.Z[14] = 1; // normal vector for x-y


    float temp = (world.X[14]*(world.X[13]-world.X[12]))
                +(world.Y[14]*(world.Y[13]-world.Y[12]))
                +(world.Z[14]*(world.Z[13]-world.Z[12]));
    float lambda = temp / ((world.X[14]*(world.X[9]-world.X[12]))
                           +(world.Y[14]*(world.Y[9]-world.Y[12]))
                           +(world.Z[14]*(world.Z[9]-world.Z[12])));
    float lambda_2 = temp / ((world.X[14]*(world.X[10]-world.X[12]))
                             +(world.Y[14]*(world.Y[10]-world.Y[12]))
                             +(world.Z[14]*(world.Z[10]-world.Z[12])));
    float lambda_3 = temp / ((world.X[14]*(world.X[4]-world.X[12]))
                             +(world.Y[14]*(world.Y[4]-world.Y[12]))
                             +(world.Z[14]*(world.Z[4]-world.Z[12])));
    float lambda_4 = temp / ((world.X[14]*(world.X[7]-world.X[12]))
                             +(world.Y[14]*(world.Y[7]-world.Y[12]))
                             +(world.Z[14]*(world.Z[7]-world.Z[12])));

    world.X[15] = world.X[12] + lambda*(world.X[9] - world.X[12]); // x component for Intersection point for p9
    world.Y[15] = world.Y[12] + lambda*(world.Y[9] - world.Y[12]); // y component for Intersection point for p9
    world.Z[15] = 0.0;

    world.X[16] = world.X[12] + lambda_2*(world.X[10] - world.X[12]);  // x component for intersection point for p10
    world.Y[16] = world.Y[12] + lambda_2*(world.Y[10] - world.Y[12]);  // x component for intersection point for p10
    world.Z[16] = 0.0;

    world.X[17] = world.X[12] + lambda_3*(world.X[4] - world.X[12]);  // x component for intersection point for p4
    world.Y[17] = world.Y[12] + lambda_3*(world.Y[4] - world.Y[12]);  // x component for intersection point for p4
    world.Z[17] = 0.0;

    world.X[18] = world.X[12] + lambda_4*(world.X[7] - world.X[12]);  // x component for intersection point for p7
    world.Y[18] = world.Y[12] + lambda_4*(world.Y[7] - world.Y[12]);  // x component for intersection point for p7
    world.Z[18] = 0.0;

    //-------------diffuse reflection-----------*
    pt_diffuse  diffuse;   //diffuse.r[3]

    //-------reflectivity coefficient-----------*
    #define 	Kdr 	0.8
    #define 	Kdg 	0.0
    #define 	Kdb 	0.0

    //--------compute distance------------------*
    float distance[UpperBD];
    for (int i=15; i<=18; i++) {
    distance[i] = sqrt(pow((world.X[i]-world.X[12]),2)+         //intersect pt p7
                        pow((world.Y[i]-world.Y[12]),2)+
                        pow((world.X[i]-world.X[12]),2) );
    //std::cout << "distance[i]  " << distance[i] << std::endl;
    }

    float angle[UpperBD], tmp_dotProd[UpperBD], tmp_mag_dotProd[UpperBD];
    for (int i = 15; i <= 18; i++) {

		tmp_dotProd[i] = world.Z[i] - world.X[12];
		printf("tep_dotProd[%d] is: %f, ", i, tmp_dotProd[i]);

		tmp_mag_dotProd[i] = sqrt(pow((world.X[i] - world.X[12]), 2) +         //[12] pt light source
									pow((world.Y[i] - world.Y[12]), 2) +
									pow((world.Z[i] - world.Z[12]), 2));
		printf("tmp_mag_dotProd[%d] is: %f, ", i, tmp_mag_dotProd[i]);

		angle[i] = tmp_dotProd[i] / tmp_mag_dotProd[i];
		printf("Angle is: %f, ", angle[i]);

		//compute color intensity
		diffuse.r[i] = Kdr * angle[i] / pow(distance[i], 2);
		printf("diffuse[%d] is: %f.\n", i, diffuse.r[i]);
		diffuse.g[i] = Kdg * angle[i] / pow(distance[i], 2);
		diffuse.b[i] = Kdb * angle[i] / pow(distance[i], 2);
	}

    float xMin = 1000.0, xMax = -1000.0;
    float yMin = 1000.0, yMax = -1000.0;

	for(int i = 0; i <= Num_index; i++)
	{
		viewer.X[i] = -sPheta * world.X[i] + cPheta * world.Y[i];
		viewer.Y[i] = -cPheta * cPhi * world.X[i] - cPhi * sPheta * world.Y[i] + sPhi * world.Z[i];
		viewer.Z[i] = -sPhi * cPheta * world.X[i] - sPhi * cPheta * world.Y[i] -cPheta * world.Z[i] + Rho;
	}

	for(int i = 0; i <= Num_index; i++)
	{
		perspective.X[i] = D_focal * viewer.X[i] / viewer.Z[i];
		perspective.Y[i] = D_focal * viewer.Y[i] / viewer.Z[i];
		//printf("Perspective [%d]:, X: %f, y: %f \n", i, perspective.X[i], perspective.Y[i]);
        if (perspective.X[i] > xMax) xMax = perspective.X[i];
        if (perspective.X[i] < xMin) xMin = perspective.X[i];
        if (perspective.Y[i] > yMax) yMax = perspective.Y[i];
        if (perspective.Y[i] < yMin) yMin = perspective.Y[i];
	}

    for(int i = 0; i <= Num_index; i++)
    {
    	int scale = 1;
        if ((xMax-xMin) != 0) perspective.X[i] = perspective.X[i]*(xMax-xMin)*scale;
        if ((yMax-yMin) != 0) perspective.Y[i] = perspective.Y[i]*(yMax-yMin)*scale;
        //std::cout << i << perspective.X[i] << perspective.Y[i] << std::endl;
    }


	for(int i = 0; i <= Num_index; i++)
		{
			perspective.X[i] = perspective.X[i] + ST7735_TFTWIDTH/2;
			perspective.Y[i] = -perspective.Y[i] + ST7735_TFTHEIGHT/2;
			//printf("After move-Perspective [%d]:, X: %f, y: %f \n", i, perspective.X[i], perspective.Y[i]);
		}
	drawLine(perspective.X[0],perspective.Y[0],perspective.X[0],perspective.Y[0], WHITE);

	drawLine(perspective.X[0],perspective.Y[0],perspective.X[1],perspective.Y[1],RED);
	drawLine(perspective.X[0],perspective.Y[0],perspective.X[2],perspective.Y[2],GREEN);
	drawLine(perspective.X[0],perspective.Y[0],perspective.X[3],perspective.Y[3],BLUE);

	//draw box

	drawLine(perspective.X[4],perspective.Y[4],perspective.X[5],perspective.Y[5],WHITE);
	drawLine(perspective.X[5],perspective.Y[5],perspective.X[6],perspective.Y[6],WHITE);
	drawLine(perspective.X[6],perspective.Y[6],perspective.X[7],perspective.Y[7],WHITE);
	drawLine(perspective.X[6],perspective.Y[6],perspective.X[4],perspective.Y[4],WHITE);

	drawLine(perspective.X[8],perspective.Y[8],perspective.X[9],perspective.Y[9],WHITE);
	drawLine(perspective.X[9],perspective.Y[9],perspective.X[10],perspective.Y[10],WHITE);
	drawLine(perspective.X[10],perspective.Y[10],perspective.X[11],perspective.Y[11],WHITE);
	drawLine(perspective.X[11],perspective.Y[11],perspective.X[8],perspective.Y[8],WHITE);

	drawLine(perspective.X[4],perspective.Y[4],perspective.X[8],perspective.Y[8],WHITE);
	drawLine(perspective.X[5],perspective.Y[5],perspective.X[9],perspective.Y[9],WHITE);
	drawLine(perspective.X[6],perspective.Y[6],perspective.X[10],perspective.Y[10],WHITE);
	drawLine(perspective.X[7],perspective.Y[7],perspective.X[11],perspective.Y[11],WHITE);

	//draw shadow
	drawLine(perspective.X[12],perspective.Y[12],perspective.X[15],perspective.Y[15],LIGHTBLUE); //p9
	drawLine(perspective.X[12],perspective.Y[12],perspective.X[16],perspective.Y[16],LIGHTBLUE); //p10
	drawLine(perspective.X[12],perspective.Y[12],perspective.X[17],perspective.Y[17],LIGHTBLUE); //p4
	drawLine(perspective.X[12],perspective.Y[12],perspective.X[18],perspective.Y[18],LIGHTBLUE); //p7

	drawLine(perspective.X[15],perspective.Y[15],perspective.X[16],perspective.Y[16],PURPLE);
	drawLine(perspective.X[17],perspective.Y[17],perspective.X[18],perspective.Y[18],PURPLE);
	drawLine(perspective.X[15],perspective.Y[15],perspective.X[17],perspective.Y[17],PURPLE);
	drawLine(perspective.X[16],perspective.Y[16],perspective.X[18],perspective.Y[18],PURPLE);

	//plot diffuse reflection
	for (int i = 15; i <= 18; i++) {
		float r, g, b;
		r = diffuse.r[i] * 15000 * 255 + 50;
		g = diffuse.g[i] * 15000 * 255 + 50;
		b = diffuse.b[i] * 15000 * 255 + 50;
		printf( "point[%d], r: %f, g: %f, b: %f \n", i, r, g, b);
		long color = (int)r << 16 | (int)g << 8 | (int)b;
		printf( "color is 0x%X \n", color);

		fillrect(perspective.X[i] - 2,perspective.Y[i] - 2,perspective.X[i] + 2,perspective.Y[i] + 2,color);
	}
}


int main (void)

{

	 uint32_t pnum = PORT_NUM;

	 pnum = 0 ;

	 if ( pnum == 0 )
		 SSP0Init();

	 else
		 puts("Port number is not correct");

	 lcd_init();

	 fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, BLACK);
/*
	 printf ("Point light is at (-50,20,100) \n");

	 float Xe, Ye, Ze, D_focal;

	 printf ("Please enter your Viewer Coordinate Xe, Ye, Ze \n");

	 printf ("Enter Xe: ");
	 scanf ("%f", &Xe);

	 printf ("Enter Ye: ");
	 scanf ("%f", &Ye);

	 printf ("Enter Ze: ");
	 scanf ("%f", &Ze);

	 printf ("Enter D_focal: ");
	 scanf ("%f", &D_focal);

	 printf ("Your entered Viewer Coordinate: (%f, %f, %f) \n", Xe, Ye, Ze);
	 printf ("D_Focal is set with %f \n", D_focal);

	 mydisplay(Xe, Ye, Ze, D_focal);

*/
	 mydisplay(300.0f, 300.0f, 300.0f, 25.0f);


/*
	 int x0,x1,y0,y1;

	 x0 = 0;
	 x1 = ST7735_TFTWIDTH/2;
	 y0 = 0;
	 y1 = ST7735_TFTHEIGHT;

	 drawLine(x0,y0,x1,y1,PURPLE);
	 */

	  return 0;

}

