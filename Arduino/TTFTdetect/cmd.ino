#define CMD(a,b) ( a + (b << 8))
#define TRUE 1
#define FALSE 0

int ProcCmd(char *pCmd)
{
  short *pCV;
  short n;
  unsigned int lv1;
  char s[22];

  pCV = (short*)pCmd;

  n = strlen(pCmd);
  if(n<2) 
          return TRUE;

  switch(*pCV)
  {           
    // sample rate   
    case ('S' + ('R'<<8)):  
    {
       sscanf(&pCmd[3],"%d",&lv1);
       srate = lv1;  // sample rate
       break;
    }

    // detection threshold
    case ('D' + ('T'<<8)):
    {
       sscanf(&pCmd[3],"%d",&lv1);
       threshold = lv1;
       break; 
    }

    // channels to record
    case ('C' + ('H'<<8)):
    {
       sscanf(&pCmd[3],"%d",&lv1);
       nchan = lv1;
       break; 
    }

    // accelerometer full-scale   
    case ('A' + ('G'<<8)):  
    {
       sscanf(&pCmd[3],"%d",&lv1);
       accelScale = lv1;  // sample rate
       break;
    }
  } 
  return TRUE;
}

void loadScript(){
  File file;
  char s[22];
  char c;
  short i;
  int16_t n;
  if(file.open("setup.txt", O_READ))
  {
    do{
        i = 0;
        s[i] = 0;
        do{
             n=file.read(&c, 1);
             if(c!='\r') s[i++] = c;
             if(i>20) break;
      }while(c!='\n');
      s[--i] = 0;
      if(s[0] != '/' && i>1){
        ProcCmd(s);
      }
    }while(n);
  file.close();  
  }
  return;  
}
