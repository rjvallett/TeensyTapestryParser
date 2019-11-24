import processing.serial.*;
import java.nio.*; 
import java.util.*;


Serial sp;

ByteBuffer sb;

Message msg;
byte[] buff;

void setup() {
  
  sp = new Serial(this, "COM23", 115200);
  
  sb = ByteBuffer.allocate(6);
  
  msg = new Message();
  buff = new byte[6];
  
  buff[0] = (byte)178;//(0x80 + 40);
  buff[1] = (byte)36;//((2 << 4) | 0x04);
  buff[2] = (byte)35;//(0x23);
  buff[3] = (byte)4;//(0x04);
  buff[4] = (byte)0;//(0x00);
  buff[5] = (byte)0;//(0x00);
  
  println(msg.parse(buff));
  
  println(msg.getAddress());
  println(msg.getEvent());
  println(msg.getData());
  
}

byte in;

void draw() {
  if (sp.available() > 0) {
    in = (byte)sp.read();
    
    if ((in & 0x80) == 0x80) {
      println((byte)(in & 0x7F));
    }
    
    /*
    sp.readBytes(buff);
    
    println(msg.parse(buff));
  
    println(msg.getAddress());
    println(msg.getEvent());
    println(msg.getData());
    //sb.put(buff);
    //sb.rewind();
    */
  }
}

enum Event {
  IDLE,     // 0
  PRESS,    // 1
  HOLD,     // 2
  RELEASE;  // 3
}

public class Message extends Observable {
  private int _address;
  private Event _event;
  private float _data;
  
  private ByteBuffer _bb;
  
  Message() {
    _bb = ByteBuffer.allocate(4);
    //_bb.order(ByteOrder.LITTLE_ENDIAN);
  } // Empty constructor
  
  // msb     lsb  event                      float data
  //  |       |      |                            |
  // [1aaa aaaa] [00ee ffff] [0fff ffff] [0fff ffff] [0fff ffff] [0fff ffff]
  //       |
  //    address
  int parse(byte[] buffer) {
    if (buffer.length != 6) {
      return -1; // Incorrect length
    }
    
    if ((buffer[0] & 0x80) == 0x80) {
      _address = (buffer[0] & 0x7F); // Address bits
      _event = Event.values()[(buffer[1] & 0x30) >> 4]; // Event bits
      
      // Data bits
      // Add data bits to the ByteBuffer
      _bb.put((byte)(((buffer[1] & 0x0F) << 4) | ((buffer[2] & 0x78) >> 3)));
      _bb.put((byte)(((buffer[2] & 0x07) << 5) | ((buffer[3] & 0x7C)) >> 2));
      _bb.put((byte)(((buffer[3] & 0x03) << 6) | ((buffer[4] & 0x7E) >> 1)));
      _bb.put((byte)(((buffer[4] & 0x01) << 7) | (buffer[5] & 0x7F)));
      _bb.rewind(); // Rewind the ByteBuffer to the starting address
      _data = _bb.getFloat(); // Set data
      _bb.rewind(); // Rewind the ByteBuffer to the starting address
      
    } else {
      return -2; // Bad header
    }
    
    return 0;
  }
  
  int getAddress() {
    return _address;
  }
  
  Event getEvent() {
    return _event;
  }
  
  float getData() {
    return _data;
  }
  
  
}
