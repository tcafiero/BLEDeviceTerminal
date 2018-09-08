// Put here function ParserPutchar to output character
void ParserPutchar(int ch)
{
  bleuart.write( (char *)&ch, 1 );
}

// Put here function ParserGetchar to input character
int ParserGetchar()
{
  return (int) bleuart.read();
}

