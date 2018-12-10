// Put here function ParserPutchar to output character
void ParserPutchar(int ch)
{
  //bleuart.write( (char *)&ch, 1 );
  bleSerial.print((char) ch);
}

// Put here function ParserGetchar to input character
int ParserGetchar()
{
  return (int) bleSerial.read();
}

