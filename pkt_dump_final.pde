import processing.serial.*;
//Serial xbee;

float ax,ay,az,gx,gy,gz,mx,my,mz;
float rot_x, rot_y, rot_z;
float DECLINATION = -11.45;
int frame[];
int f_index;

PrintWriter g_output;
PrintWriter a_output;
PrintWriter m_output;
PrintWriter r_output;

void setup(){
  size(200,200,P3D);
  rectMode(CENTER);
  stroke(255);
  Serial xbee = new Serial(this,"/dev/cu.usbserial-DN02ZC6S",9600);
  xbee.buffer(128); // this is a buffer so for each event, this many bytes is read off
  xbee.clear();
  frame = new int[128];
  f_index = 0;
  
  String timeString = year()+"-"+month()+"-"+day()+"-"+hour()+minute()+".csv";
  g_output = createWriter("g_data-"+timeString);
  a_output = createWriter("a_data-"+timeString);
  m_output = createWriter("m_data-"+timeString);
  r_output = createWriter("r_data-"+timeString);
  g_output.println("x,y,z");
  a_output.println("x,y,z");
  m_output.println("x,y,z");
  
}
float get4bytesFloat(int[] data, int offset) { 
  String hexint=hex(data[offset+3],2)+hex(data[offset+2],2)+hex(data[offset+1],2)+hex(data[offset],2);
  return Float.intBitsToFloat(unhex(hexint)); 
}
int get2bytesInt(int[] data, int offset) { 
  String hexint=hex(data[offset+1],2)+hex(data[offset],2);
  return unhex(hexint); 
}

void draw(){
  translate(100, 100, 0);

  float rotx = (rot_x/360.0)*2*PI+PI;
  float roty = (rot_y/420.0)*2*PI+PI;
  float rotz = (rot_z/360.0)*-2*PI+PI;
  float roll = atan2(ay,-az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading=0;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  rotateX(pitch);
  rotateY(roll);
  rotateZ(heading);
  //println("Pitch, Roll: ",pitch* 180.0/PI,roll*180/PI);
  //println("Heading:",heading*180/PI);
  fill(51);
  background(100);

  rect(0, 0, 100, 100);
  
}
void processFrame(int[] frame){
  //all of this information is derived from the digi documentation
  //http://docs.digi.com/display/RFKitsCommon/Frame+structure
  
  //extract all of the header info from the frame
  int start_delimiter = frame[0];
  int frame_length = unhex(hex(frame[1],2)+hex(frame[2],2));
  int frame_type = frame[3];
  int _16_bit_source_addr = unhex(hex(frame[4],2)+hex(frame[5],2));
  int rssi = frame[6];
  int options = frame[7];
  
  //if the frame_length says 0, something is up
  if(frame_length<1) frame_length = 6;//technically, this is a hack
  
  //data says int, but its a byte array
  int data[] = new int[frame_length-5];//five of the bytes of frame_length
  for(int i=0;i<frame_length-5;i++){//populate the data byte array
    data[i] = frame[8+i];
    //print(data[i]);
  } 
  
  if(data[0]=='I'){//this is the IMU data
    //println(hex(data[0],2));
    gx = get4bytesFloat(data,1);//offset by one because data[0] is 'I'
    gy = get4bytesFloat(data,5);
    gz = get4bytesFloat(data,9);
    //println(gx,gy,gz);
    ax = get4bytesFloat(data,13);//offset by one because data[0] is 'I'
    ay = get4bytesFloat(data,17);
    az = get4bytesFloat(data,21);
    //println(ax,ay,az);
    mx = get4bytesFloat(data,25);//offset by one because data[0] is 'I'
    my = get4bytesFloat(data,29);
    mz = get4bytesFloat(data,33);
    //println(mx,my,mz);
    g_output.println(gx+","+gy+","+gz);
    a_output.println(ax+","+ay+","+az);
    m_output.println(mx+","+my+","+mz);
  }
  if(data[0]=='R'){
    //R for respiratory signal
    int r = get2bytesInt(data,1);
    println(r);
    r_output.println(r);
  }
}
void serialEvent(Serial xbee) {
  //each single byte is it's own event!
  // this functiong gets called once for every [buffer size] bytes that comes in
  //hopefully, there is no interleaving between the data (IMU vs. Resp)
  for(int i=0;i<128;i++){
    int my_byte = xbee.read(); 
    if(my_byte == 0x7E){ //we have a byte that is a "start frame" delimiter
      processFrame(frame); //process the previous frame
      frame[0] = my_byte;//start building up the next frame
      f_index=1;//bump the f_index
    } else if(f_index!=0) { //we're in the middle of processing a frame
      frame[f_index]=my_byte;//write the incoming byte to the frame
      f_index++;//bump the frame index
    }
  }
}

void keyPressed() {
  g_output.flush(); // Writes the remaining data to the file
  g_output.close(); // Finishes the file
  a_output.flush(); // Writes the remaining data to the file
  a_output.close(); // Finishes the file
  m_output.flush(); // Writes the remaining data to the file
  m_output.close(); // Finishes the file
  r_output.flush();
  r_output.close();
  exit(); // Stops the program
}