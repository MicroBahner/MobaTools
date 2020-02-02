////////////////////////////////////////////////////////////////////////////
// MoToTimer.h - part of MobaTools
//
// Class MoToTimer - Timerverwaltung für Zeitverzögerungen in der Loop-Schleife
// 
class MoToTimer
{
  public:
    MoToTimer() {
        active = false;
    }

    void setTime(  long wert ) {
        endtime =  (long) millis() + ( (long)wert>0?wert:1 );
        active = true;
    }

    bool running() {
        if ( active ) active =  ( endtime - (long)millis() > 0 );
        return active;
    }

    bool expired() { return !running(); }

    void stop() { active = false; }

    long getTime() {
        // return remaining time
        if ( running() ) return endtime - (long)millis();
        else return 0;
    }
  private:
    bool active;
    long endtime;
};
