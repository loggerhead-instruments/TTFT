
void getTime(){
  day = rtc.getDay();
  month = rtc.getMonth();
  year = rtc.getYear();
  hour = rtc.getHours();
  minute = rtc.getMinutes();
  second = rtc.getSeconds();
}

// Calculates Accurate UNIX Time Based on RTC Timestamp
unsigned long RTCToUNIXTime(int uYear, int uMonth, int uDay, int uHour, int uMinute, int uSecond){
  int i;
  unsigned const char DaysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  unsigned long Ticks = 0;

  long yearsSince = uYear+30; // Same as tm->year + 2000 - 1970
  long numLeaps = yearsSince >> 2; // yearsSince / 4 truncated
  
  if((!(uYear%4)) && (uMonth>2)) Ticks+=SECONDS_IN_DAY;  //dm 8/9/2012  If current year is leap, add one day

  // Calculate Year Ticks
  Ticks += (yearsSince-numLeaps)*SECONDS_IN_YEAR;
  Ticks += numLeaps * SECONDS_IN_LEAP;

  // Calculate Month Ticks
  for(i=0; i < uMonth-1; i++){
       Ticks += DaysInMonth[i] * SECONDS_IN_DAY;
  }

  // Calculate Day Ticks
  Ticks += uDay * SECONDS_IN_DAY;
  
  // Calculate Time Ticks CHANGES ARE HERE
  Ticks += (uint32_t)uHour * SECONDS_IN_HOUR;
  Ticks += (uint32_t)uMinute * SECONDS_IN_MINUTE;
  Ticks += uSecond;

  return Ticks;
}
