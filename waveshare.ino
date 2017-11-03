
void paintwaveshare() {   
      int rc=60; //round corner diameter
      int xres=200;
      int yres=200;
      float alpha;

// scale vario to angle
      alpha =PI* avg /(2* 3.6);        
// clear screen  
     waveshare.fillRect(0,0,xres,yres,BLACK);        
  
//draw edge
       projectborder(rc, xres,yres,alpha, 0);
       if (alpha>PI/2) y=0;
       if (alpha<-PI/2) y=200;
       waveshare.fillRect(0,100,max(x,20),y-100,WHITE);
       waveshare.fillRoundRect(5, 5, 190, 190, rc-10, BLACK); 
 
       paintsharpnumbers(rc);    
      
// cirle indicator
      projectborder(rc, xres,yres,-alpha /2 - 0.3, 8);
      waveshare.fillCircle(x, y, 8, WHITE);  
      waveshare.fillCircle(x, y, 4, BLACK); 
      
//triangle 
      float triangle=0.30; //angle of triangle needle in rad
      projectborder(rc,xres,yres,alpha, 2);
      float x1=x+20*(cos(alpha-triangle));
      float y1=y+20*(sin(alpha-triangle));
      float x2=x+20*(cos(alpha+triangle));
      float y2=y+20*(sin(alpha+triangle));
      waveshare.fillTriangle(x, y, x1,y1,x2,y2, WHITE);    
      waveshare.updateWindow(0, 0, 200, 200, false);
      
     // waveshare.refresh();
//      previousMillis[0]=millis();
}


void paintsharpnumbers(int rc) {
      int ndigits=floor(log10(abs(alt)))+1;
      int digitwidth=22;
  
 
    // top numbers
      waveshare.setTextSize(1);
      waveshare.setTextColor(WHITE,BLACK);
      waveshare.setFont(&Lato_Regular_44);
  //ndigits=4;
      
      waveshare.setCursor(130-ndigits*digitwidth,58);
      
      waveshare.print(int(alt));
     
    //waveshare.print(vario);
      waveshare.setFont(&DejaVu_Sans_Mono_18);
      waveshare.print("m");    

      // bottom numbers
      waveshare.setCursor(45,170);
      waveshare.setFont(&Lato_Regular_44);
      waveshare.print(stf);    
      waveshare.setFont(&DejaVu_Sans_Mono_18);
      waveshare.print("KmH"); 
   
      // central small numbers
      waveshare.setFont(&DejaVu_Sans_Mono_18);
      waveshare.setCursor(120,113);
      waveshare.print("m/s"); 
      waveshare.setCursor(120,92);
      waveshare.print("5s");  

       // central number
      waveshare.setCursor(31,113);
      waveshare.setTextColor(WHITE);
      waveshare.setFont(&Lato_Regular_44);
                 
      // STF indicator
      int counter=0;
      int xstart, ystart;
      waveshare.fillCircle(185, 100, 10, WHITE);
      if (avg >=-0.0001) {      
        int16_t xpos, ypos;
        uint16_t w,h;
       // waveshare.getTextBounds(string2char("test of some text"), 30, 119, &xpos, &ypos, &w, &h);       
        waveshare.print("+");  
        ystart=83;
        while (ystart>0 && counter <=avg-1 && counter <5) {
          waveshare.fillTriangle(170,ystart, 200,ystart,185,ystart-10,WHITE);
          ystart-=14;
          counter++;
        }                
      } else {
        waveshare.print("-");  
        ystart=117;
        while (ystart<168 && counter <=abs(avg)-1&& counter <5) {
          waveshare.fillTriangle(170,ystart, 200,ystart,185,ystart+10,WHITE);
          ystart+=14;
          counter++;
        }       
      }     
        
     
      waveshare.setCursor(55,113);
      if (abs(avg)<10) waveshare.println(abs(avg),1);  
      else waveshare.println(abs(avg),0);   

      
      for (int x = 0; x<rc; x++) {
          // draw outside border
  //      waveshare.drawRoundRect(0, 0, 200, 200, rc-x, WHITE); 
 
      }    
      
}


