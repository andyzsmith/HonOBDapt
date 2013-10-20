





class HondaRequest
{
  public:
  
  int Address;
  int Length;
  
  HondaRequest(int _Address, int _Length)
  {
    Address = _Address;
    Length = _Length;
  }
  
  HondaRequest() { } 
  
};


class HondaCommMessageProcessor
{
  private:
  
  enum Shake { received, processed };
  Shake _State;
  
  public:
  
  void Reset();
  void Work(byte* Message);
  boolean DoneWorking();
};

  
class HondaCommMessageCollector
{
  private:
  int HondaMsgIdx;
  int hondabyte;
  boolean burst;
  int burst_limit;
  long start_collecting_tod;
  
  enum Shake { comm_estab, collect, complete };
  Shake _State;

public:

void Reset();
void Work(byte* Message, int ExpectedLen);
boolean DoneWorking();
}
;



class HondaCommCoordinator
{

  private:
  enum Shake { gotwork, working, received, processed };
  Shake _State;
  HondaCommMessageCollector oMessageCollector;
  HondaCommMessageProcessor oMessageProcessor;
  
  byte _Message[8];
  
  HondaRequest _HondaRequest;



public:

void Reset();
void AssignWork(HondaRequest request);
void Work();
};
