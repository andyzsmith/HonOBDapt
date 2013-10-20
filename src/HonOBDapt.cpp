
/* this is a block comment */

//#include <SoftwareSerial.h>
#include <SoftwareSerialWithHalfDuplex.h>
#include <Arduino.h>
#include "/home/andrew/HonOBDapt/HondaCommCoordinator.h"
const int SERIAL_BURST_MAX_TIME_SLICE = 5000;


enum eProtocol { ISO14230_Slow_Init, ISO14230_Fast_Init, ISO9141 };
eProtocol Protocol = ISO14230_Fast_Init;


class ISpecialBaudChecksumPort
{ 
  public:
  virtual int _read() = 0;
  virtual void _write(int in) = 0;
  virtual void _reset_c() = 0;
  virtual void _begin() = 0;
  virtual boolean _valid_checksum(int InputChecksum) = 0;
  virtual void _compute_send_c() = 0;
}
;

class GenericChecksumPort : public SoftwareSerialWithHalfDuplex, ISpecialBaudChecksumPort
{
  private:
  
  int _checksum;
  
  public:
  
  GenericChecksumPort( int InPin, int OutPin, boolean InvertedLogic, boolean HalfDuplex ) :
    SoftwareSerialWithHalfDuplex ( InPin, OutPin, InvertedLogic, HalfDuplex )
    {
    }
    
  virtual int _compute_c(int input_sum) = 0;
  
  int _read()
  {
    int _byte = this -> read();
    _checksum += _byte;
    return _byte;
  }
  
  boolean _valid_checksum(int InputChecksum)
  {
  return (   InputChecksum == _compute_c(  _checksum - InputChecksum  )     );
  }
  
  void _write(int in)
  {
    _checksum += in;
    this -> write(in);
  }
  
  void _reset_c()
  {
    _checksum = 0;
  }
  
  void _compute_send_c()
  {
    this -> write( _compute_c( _checksum ) );
  }
};


class _ISOPort : public GenericChecksumPort
{
  private:
  
  int _compute_c(int input_sum)
  {
   input_sum = input_sum % 256;
   return input_sum;
  }
  
  public:
  
  void _begin()
  {
    this -> begin(10400);
  }
  
  _ISOPort(int InPin, int OutPin) : GenericChecksumPort(InPin, OutPin, false, true)
  {
    this->_reset_c();
  }
  

};

class _HondaPort : public GenericChecksumPort
{
  private:
  
  int _compute_c(int input_sum)
  {
    if (input_sum > 256)
      while (input_sum > 256) input_sum-=256;
    
    input_sum = 256-input_sum;
    return input_sum;
  }
  
  public:
  
  void _begin()
  {
    this -> begin(9470);
  }
  
  _HondaPort(int InPin, int OutPin) : GenericChecksumPort( InPin, OutPin, false, false )
  {
    this->_reset_c();
  }
};


_ISOPort  ISOPort( 6, 8 );

_HondaPort HondaPort( 10, 10 ) ;


void HondaCommCoordinator::AssignWork(HondaRequest aHondaRequest)
  {
    _State = gotwork;
    this -> _HondaRequest = aHondaRequest;
  }
  
  void HondaCommCoordinator::Reset()
  {
    _State = processed;
    oMessageCollector.Reset();
    oMessageProcessor.Reset();
  }
  

  void HondaCommCoordinator::Work()
  { 
    if (_State == gotwork)
    {
    //  HondaPort.listen();
      _State = working;
    }
    
    if (_State == working)
    { 
      oMessageCollector.Work(_Message, _HondaRequest.Length);
      if (oMessageCollector.DoneWorking()) 
      {  
        _State = received;
      }
    }
    
    if (_State == received)
    { 
        oMessageProcessor.Work(_Message);
        if (oMessageProcessor.DoneWorking())
        {
          _State = processed;
          oMessageCollector.Reset();
          oMessageProcessor.Reset();
          ISOPort.listen();
        }
    }
  }







void HondaCommMessageCollector::Work(byte* Message, int ExpectedLen)
  {  
    if (_State == comm_estab || _State == collect)
    {
      if (_State == comm_estab)
      {
        HondaPort._reset_c();
        start_collecting_tod = millis();
        HondaMsgIdx = 0;
        burst = false;
      }
      
      if (_State == collect)
      {
        if (  (millis() - start_collecting_tod) > 5000)
        {
          this->Reset();
          Serial.println("honda communications message collecting timeout");
        }
      }
    
      do {
  
            if( HondaPort.available() )
            {
                hondabyte = HondaPort._read();
                Message[HondaMsgIdx] = hondabyte;
                HondaMsgIdx++;
                _State = collect;
                if (!burst)
                {
                  burst = true;
                  burst_limit = 0;
                }
            }
            if (_State == collect)
            {
              burst_limit++;
              if ( HondaMsgIdx == ExpectedLen + 3 ) 
              { 
                  if (  HondaPort._valid_checksum(hondabyte) &&  Message[0] == 0  )
                      { 
                        burst = false;
                        _State = complete;
                      } else { Serial.println("Honda Data Length OK Checksum Wrong"); } 
              }             
            }
      } while (burst && (burst_limit < SERIAL_BURST_MAX_TIME_SLICE));
      
    }
  }


int SHAKE_LEVEL = 0;
const int SHAKE_NO_COMM = 0;
const int SHAKE_COMM_ESTAB = 2;

const int KWP_SLOW_INIT_HEAD = 0x33;
const int SAE_J1979_HEAD = 0x68;
const int OBDII_HEAD = 0x6A;
const int SCAN_HEAD = 0xF1;
const int MODE1_HEAD = 0x01;
const int MODE2_HEAD = 0x02;
const int SCANNER_COMMAND_BYTE_REQUEST_CAPABILITIES = 0;

unsigned long simcount;
int ramp;



boolean valid_mode(byte modebyte)
{
  return (modebyte > 0 && modebyte < 10);
}

boolean keep_alive(byte modebyte)
{
  return (modebyte == 0x3E);
}

class HondaMap
{
  public :
  int Address;
  int Length;
  HondaMap(int _Address, int _Length)
  {
    Address = _Address;
    Length = _Length;
  }
};

class ISOPID
{
  public :
  int Mode;
  int PID;
  
    ISOPID(int _Mode, int _PID)
    {
      Mode = _Mode;
      PID = _PID;
    }
    
    boolean equls(ISOPID __compare)
    {
      return (  (__compare.Mode == this->Mode) && (__compare.PID == this->PID) );
    }
};

const ISOPID ISO_PID_CAPABILITIES(1, 0x00);
const ISOPID ISO_PID_ENGINE_RPM(1,0x0C);
const ISOPID ISO_PID_O2_SENSOR_VOLTAGE(1,0x08);
const ISOPID ISO_PID_COOLANT_TEMP(1, 0x05);
const ISOPID ISO_PID_SHORT_TERM_FUEL_TRIM(1, 0x06);
const ISOPID ISO_PID_LONG_TERM_FUEL_TRIM(1, 0x07);
const ISOPID ISO_PID_MIL(1, 0x01);
const ISOPID ISO_PID_INTAKE_TEMP(1, 0x0F);
const ISOPID ISO_PID_SPARK_ADVANCE(1, 0x0E);
const ISOPID ISO_PID_THROTTLE_POSITION(1, 0x11);
const ISOPID ISO_PID_OBD_COMPLIANCE_LEVEL(1, 0x1C);
const ISOPID ISO_PID_FUEL_SYSTEM_STATUS(1, 0x03);
const ISOPID ISO_PID_DTCs ( 3, 0);



HondaCommCoordinator oHondaCommCoord;

class IGenericAsyncHandler 
{
  public:
  virtual void Callback(byte* data)=0;
};


class IHandler
{ 
   public :    
   virtual void IncomingPID(ISOPID c)=0;
};

class IGenericHandler : public IHandler
{
  public :
  virtual char* VIN() = 0;
  virtual char* Capabilities(ISOPID c)=0;
};

class IISOReply
{
  public:
  virtual void ISOReply( byte* _ISO_OUT_MSG, int _ISO_OUT_LEN, ISOPID ORIGINAL_REQUEST_PID ) = 0;
  virtual void WriteFormatByte(int _ISO_OUT_LEN) = 0;
  virtual void WriteTargetAddressByte() = 0;
};

class GenericISOReply : public IISOReply
{
  public:
  void ISOReply( byte* _ISO_OUT_MSG, int _ISO_OUT_LEN, ISOPID ORIGINAL_REQUEST_PID  ) 
  { 
    ISOPort._reset_c();
    
    if (ORIGINAL_REQUEST_PID.Mode == 3)
      WriteFormatByte(_ISO_OUT_LEN-1);
    else
      WriteFormatByte(_ISO_OUT_LEN);
      
    WriteTargetAddressByte();

    ISOPort._write(0x40); //ECU ADDRESS
    
    ISOPort._write(0x40 | ORIGINAL_REQUEST_PID.Mode); 
    
    if (ORIGINAL_REQUEST_PID.Mode != 3)
      ISOPort._write(ORIGINAL_REQUEST_PID.PID);

    for ( int _MSGIDX = 0; _MSGIDX < _ISO_OUT_LEN; _MSGIDX++)
    {
      ISOPort._write(_ISO_OUT_MSG[_MSGIDX]);
    }           
    ISOPort._compute_send_c();
  }
};


class ProtocolReply_ISO14230 : public GenericISOReply
{
  public:
  void WriteFormatByte(int _ISO_OUT_LEN)
  {
     ISOPort._write(0b10000000 | ( _ISO_OUT_LEN + 2) );
  }
  void WriteTargetAddressByte()
  {
     ISOPort._write(0xF1);
  }
};

class ProtocolReply_ISO9141 : public GenericISOReply
{
    public:
    void WriteFormatByte(int _ISO_OUT_LEN)
    {
      ISOPort._write(0x48); 
    }
    void WriteTargetAddressByte()
    {
      ISOPort._write(0x6B);
    }
};

ProtocolReply_ISO9141 oProtocolReply_ISO9141;
ProtocolReply_ISO14230 oProtocolReply_ISO14230;

class GenericHandler : public IGenericHandler
{
  private:
  IISOReply* _ISOReply;
  public:
  GenericHandler()
  {
    if (Protocol == ISO9141)
    {
      _ISOReply = &oProtocolReply_ISO9141;
    }
    else
    {
      _ISOReply = &oProtocolReply_ISO14230;
    }
  }
  
  virtual void IncomingPID(ISOPID t) { } ;
  virtual char* VIN() { }
  virtual char* Capabilities(ISOPID ISOPID) { }
  virtual int CoolantTemp(ISOPID ISOPID) { } 
  virtual int EngineRPM(ISOPID ISOPID) { } 
  virtual int O2Voltage(ISOPID ISOPID) 
  {
    int O2 = 50;
    ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID, O2);
  }
  virtual int ShortTermFuelTrim(ISOPID ISOPID) { }
  virtual int MIL(ISOPID ISOPID) { }
  virtual int InjectorWidth(ISOPID ISOPID) { }
  virtual int IntakeTemp(ISOPID ISOPID) { } 
  virtual int SparkAdvance(ISOPID ISOPID) { }
  virtual int ThrottlePosition(ISOPID ISOPID) { }
  virtual void OBDCompliance(ISOPID ISOPID) { }
  virtual void FuelSystemStatus(ISOPID ISOPID) { }
  virtual void DTCs(ISOPID ISOPID) { }
  
  
  void ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID ISOPID, byte ISOMeasure)
  {
    byte ISOMSG[10];
    int  ISOLEN;
    ISOLEN = 1;
    ISOMSG[0] = ISOMeasure;
    _ISOReply->ISOReply( ISOMSG, ISOLEN, ISOPID );
  }
  
  void ISO_Measurements_to_ISO_Reply_scalar_word(ISOPID ISOPID, int ISOMeasure)
  {
    byte ISOMSG[10];
    int  ISOLEN;
    ISOLEN  = 2;
    ISOMSG[0] = ISOMeasure / 256;
    ISOMSG[1] = ISOMeasure % 256;
    _ISOReply->ISOReply( ISOMSG, ISOLEN, ISOPID );
  }
  
  void ISO_Measurements_to_ISO_Reply_double_word_bit_encoded_matrix(ISOPID ISOPID, byte* ISOMatrix)
  {
    byte ISOMSG[10];
    int  ISOLEN;
    ISOLEN  = 4;
    ISOMSG[0] = ISOMatrix[0];
    ISOMSG[1] = ISOMatrix[1];
    ISOMSG[2] = ISOMatrix[2];
    ISOMSG[3] = ISOMatrix[3];
    _ISOReply->ISOReply( ISOMSG, ISOLEN, ISOPID );
  }
  
  void ISO_Measurements_to_ISO_Reply_six_byte_matrix(ISOPID ISOPID, byte* ISOMatrix)
  {
    byte ISOMSG[10];
    int  ISOLEN;
    ISOLEN  = 6;
    ISOMSG[0] = ISOMatrix[0];
    ISOMSG[1] = ISOMatrix[1];
    ISOMSG[2] = ISOMatrix[2];
    ISOMSG[3] = ISOMatrix[3];
    ISOMSG[4] = ISOMatrix[4];
    ISOMSG[5] = ISOMatrix[5];
    _ISOReply->ISOReply( ISOMSG, ISOLEN, ISOPID );
  }
  
  void ISOPID_TO_GENERIC_HANDLER_MAP(ISOPID __ISOPID)
  {
    if (__ISOPID.equls(ISO_PID_ENGINE_RPM))
    {
      EngineRPM(__ISOPID);
    } 
    else
    if (__ISOPID.equls(ISO_PID_O2_SENSOR_VOLTAGE))
    {
      O2Voltage(__ISOPID);
    }
    else
    if (__ISOPID.equls(ISO_PID_COOLANT_TEMP))
    {
      CoolantTemp(__ISOPID);
    }
    else
    if (__ISOPID.equls(ISO_PID_CAPABILITIES))
    {
      Capabilities(__ISOPID);
    }
    else
    if (__ISOPID.equls(ISO_PID_SHORT_TERM_FUEL_TRIM))
    {
      ShortTermFuelTrim(__ISOPID);
    }
    else
    if (__ISOPID.equls(ISO_PID_MIL))
    {
      MIL(__ISOPID);
    }
    //else
    //if (__ISOPID == ISO_PID_INJECTORWIDTH)
    //{
    //  InjectorWidth(__ISOPID);
    //}
    else
    if (__ISOPID.equls(ISO_PID_INTAKE_TEMP))
    {
      IntakeTemp(__ISOPID);
    }
    else
    if (__ISOPID.equls(ISO_PID_SPARK_ADVANCE))
    {
      SparkAdvance(__ISOPID);
    }
    else
    if (__ISOPID.equls(ISO_PID_THROTTLE_POSITION))
    {
      ThrottlePosition(__ISOPID);
    }
    else
    if (__ISOPID.equls(ISO_PID_OBD_COMPLIANCE_LEVEL))
    {
      OBDCompliance(__ISOPID);
    }
    else
    if (__ISOPID.equls(ISO_PID_FUEL_SYSTEM_STATUS))
    {
      FuelSystemStatus(__ISOPID);
    }
    else
    if (__ISOPID.Mode == ISO_PID_DTCs.Mode)
    {
      DTCs(__ISOPID); 
    }
  }
};


const HondaMap HONDA_RPM(0x00, 2);
const HondaMap HONDA_O2(0x15, 1);
const HondaMap HONDA_COOLTEMP(0x10, 1);
const HondaMap HONDA_SPARKADVANCE(38, 1);
const HondaMap HONDA_SHORT_TERM_FUEL_TRIM(32,1);
const HondaMap HONDA_LONG_TERM_FUEL_TRIM(34,1);
const HondaMap HONDA_MIL(0x05,1);
const HondaMap HONDA_INJECTOR(26, 2);
const HondaMap HONDA_INTAKETEMP(17, 1);
const HondaMap HONDA_THROTTLE_POSITION(20, 1);
//const HondaMap HONDA_FUEL_SYSTEM_STATUS(

class HondaHandler : public GenericHandler, public IGenericAsyncHandler
{
  private:
  
  int originalmode;
  int originalpid;
  
  
  void (HondaHandler::*callback_pointer)(byte *);
  
  public:
  
  void Callback(byte* ready_data)
  {
      (this->*callback_pointer)( ready_data );
  }
  
  float ThermistorToCelcius ( int ThermistorReading )
  {
    float CoolScaled = ( 5.0F / 256.0F ) * ThermistorReading;
    float CoolFHeit = 
    ( pow(CoolScaled, 6.0F ) *.1423F) - 
    ( pow(CoolScaled, 5.0F ) * 2.4938F) +
    ( pow(CoolScaled, 4.0F ) * 17.837F) - 
    ( pow(CoolScaled, 3.0F ) * 68.698F) +
    ( pow(CoolScaled, 2.0F ) * 154.69F) -
    ( CoolScaled * 232.75F ) +
    291.24F
    ;
    float CoolCelcius = (CoolFHeit - 32.0F) / 1.8F;
    return CoolCelcius;
  }
  
  char* Capabilities(ISOPID isopid)
  {
    byte HondaCapabilities[4];

    unsigned long capabilities_bits = 0;
    
    bitSet( capabilities_bits, 32 - ISO_PID_ENGINE_RPM.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_COOLANT_TEMP.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_O2_SENSOR_VOLTAGE.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_SHORT_TERM_FUEL_TRIM.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_LONG_TERM_FUEL_TRIM.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_MIL.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_INTAKE_TEMP.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_SPARK_ADVANCE.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_THROTTLE_POSITION.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_MIL.PID );
    bitSet( capabilities_bits, 32 - ISO_PID_OBD_COMPLIANCE_LEVEL.PID );
    
    HondaCapabilities[3] = ( capabilities_bits & 0b00000000000000000000000011111111 );
    HondaCapabilities[2] = (( capabilities_bits >> 8 ) & 0b00000000000000000000000011111111 );
    HondaCapabilities[1] = (( capabilities_bits >> 16 ) & 0b00000000000000000000000011111111 );
    HondaCapabilities[0] = (( capabilities_bits >> 24 ) & 0b00000000000000000000000011111111 );
    
    ISO_Measurements_to_ISO_Reply_double_word_bit_encoded_matrix(isopid, HondaCapabilities);
  }
  
  int ThrottlePosition(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_THROTTLE_POSITION.Address, HONDA_THROTTLE_POSITION.Length, &HondaHandler::Async_Callback_ThrottlePosition);
  }
  
  void Async_Callback_ThrottlePosition(byte* HondaReadyData)
  {
    int ISOMeasure = 0;
    
    byte ThrottlePosition = HondaReadyData[3];
    
    ISOMeasure = (ThrottlePosition*100.0F/255.0F);
    
    ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID(originalmode, originalpid), ISOMeasure); 
  } 
  
  
  int SparkAdvance(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_SPARKADVANCE.Address, HONDA_SPARKADVANCE.Length, &HondaHandler::Async_Callback_SparkAdvance);
  }
  
  void Async_Callback_SparkAdvance(byte* HondaReadyData)
  {
    int ISOMeasure = 0;
    
    byte SparkAdvance = HondaReadyData[3];
    float SparkAdvanceDegress = ( SparkAdvance * .234375F ) - 4.95F;
    
    ISOMeasure = (SparkAdvance/2) - 64;
    
    ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID(originalmode, originalpid), ISOMeasure); 
  } 
  
  
    void FuelSystemStatus(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_INJECTOR.Address, HONDA_INJECTOR.Length, &HondaHandler::Async_Callback_FuelSystemStatus);
  }
  
  void Async_Callback_FuelSystemStatus(byte* HondaReadyData)
  {
    int ISOMeasure = 0;
    
    byte OpenLoop = HondaReadyData[3];

    enum OpenLoopClosedLoop { OpenLoopCold, ClosedLoop, OpenLoopDemand, OpenLoopFailure, ClosedLoopFailure };

  OpenLoopClosedLoop FuelState = ClosedLoop;

  byte ISOByte = 0;
  if (FuelState == OpenLoopCold)
    ISOByte |= 0x00000001;
  if (FuelState == ClosedLoop)
    ISOByte |= 0x00000010;
  if (FuelState == OpenLoopDemand)
    ISOByte |= 0x00000011;
  if (FuelState == OpenLoopFailure)
    ISOByte |= 0x00000100;
  if (FuelState == ClosedLoopFailure)
    ISOByte |= 0x00000101;
   
    
    ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID(originalmode, originalpid), ISOByte); 
  } 
  
  
  int InjectorWidth(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_INJECTOR.Address, HONDA_INJECTOR.Length, &HondaHandler::Async_Callback_InjectorWidth);
  }
  
  void Async_Callback_InjectorWidth(byte* HondaReadyData)
  {
    int ISOMeasure = 0;
    
    byte InjectorHi = HondaReadyData[3];
    byte InjectorLow = HondaReadyData[4];
    //( ( RPM_HI * 256 ) + RPM_LOW ) ;
    //byte O2Voltage = HondaReadyData[3];
    ISOMeasure = 0;
    
    ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID(originalmode, originalpid), ISOMeasure); 
  }  
  
  
  
  int IntakeTemp(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_INTAKETEMP.Address, HONDA_INTAKETEMP.Length, &HondaHandler::Async_Callback_IntakeTemp);
  }

  void Async_Callback_IntakeTemp(byte* HondaReadyData)
  {
    int ISOMeasure = 0;
    byte IntakeThermistor = HondaReadyData[3];
    
    float IntakeCelcius = ThermistorToCelcius( IntakeThermistor );

    ISOMeasure = IntakeCelcius - 40;
    
    ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID(originalmode, originalpid), ISOMeasure); 
  }
  
  
  
  int CoolantTemp(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_COOLTEMP.Address, HONDA_COOLTEMP.Length, &HondaHandler::Async_Callback_CoolantTemp);
  }

  void Async_Callback_CoolantTemp(byte* HondaReadyData)
  {
    int ISOMeasure = 0;
    byte CoolantThermistor = HondaReadyData[3];
    
    float CoolCelcius = ThermistorToCelcius( CoolantThermistor );

    ISOMeasure = CoolCelcius - 40;
    
    ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID(originalmode, originalpid), ISOMeasure); 
  }
  
  int O2Voltage(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_O2.Address, HONDA_O2.Length, &HondaHandler::Async_Callback_O2Voltage);
  }
  
  void Async_Callback_O2Voltage(byte* HondaReadyData)
  {
    int ISOMeasure = 0;
    byte O2Voltage = HondaReadyData[3];
    ISOMeasure = O2Voltage;
    ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID(originalmode, originalpid), ISOMeasure); 
  }  
  
  int EngineRPM(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_RPM.Address, HONDA_RPM.Length, &HondaHandler::Async_Callback_EngineRPM);
  }
  void Async_Callback_EngineRPM(byte* HondaReadyData)
  {  
      int ISOMeasure = 0;
    
      byte RPM_HI = HondaReadyData[3];
      byte RPM_LOW = HondaReadyData[4];

      long RPM = 1875000 /  ( ( RPM_HI * 256 ) + RPM_LOW ) ;
      ISOMeasure = RPM / 4;
      
      ISO_Measurements_to_ISO_Reply_scalar_word(ISOPID(originalmode, originalpid), ISOMeasure); 
  }
  
  int ShortTermFuelTrim(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_SHORT_TERM_FUEL_TRIM.Address, HONDA_SHORT_TERM_FUEL_TRIM.Length, &HondaHandler::Async_Callback_ShortTermFuelTrim);
  }
  
  void Async_Callback_ShortTermFuelTrim(byte* HondaReadyData)
  {
    int ISOMeasure = 0;
    byte ShortTermFuelTrim = HondaReadyData[3];
    int signedSTFT = ShortTermFuelTrim - 128;
    
    ISOMeasure = signedSTFT + 128;

    ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID(originalmode, originalpid), ISOMeasure); 
  }  
  
  void OBDCompliance(ISOPID isopid)
  {
    byte ISOByte = 0;
    
    boolean OBDI = false;
    boolean OBDII = true;
    boolean NonOBD = false;
    
    if (OBDI)
      ISOByte |= 0b00000100;
      
    if (OBDII)
      ISOByte |= 0b00000001;
     
     if (NonOBD)
       ISOByte |= 0b00000101;
       
     ISO_Measurements_to_ISO_Reply_scalar_byte(ISOPID(originalmode, originalpid), ISOByte); 

  }
  
  
  int MIL(ISOPID isopid)
  {
    GeneralAsyncRequest( isopid, HONDA_MIL.Address, HONDA_MIL.Length, &HondaHandler::Async_Callback_MIL);
  }
  
  void Async_Callback_MIL(byte* HondaReadyData)
  {
    byte ISOStructure[4];
    
    int DTCCount = 3;
    
    boolean MILOn = true;
    
    byte ISOByte1 = 0b00000000 | DTCCount;
    
    if (MILOn)
      ISOByte1 = ISOByte1 | 0b10000000;
    
    byte ISOByte2 = 0;
    boolean MisfireTestAvailable = false;
    boolean MisfireTestComplete = false;
    boolean FuelTestAvailable = false;
    boolean FuelTestComplete  = false;
    boolean ComponentsTestAvailable  = false;
    boolean ComponentsTestComplete  = false;
    if (MisfireTestAvailable)
      ISOByte2 = ISOByte2 | 0b00000001;
    if (!MisfireTestComplete)
      ISOByte2 = ISOByte2 | 0b00010000;
    if (FuelTestAvailable)
      ISOByte2 = ISOByte2 | 0b00000010;
    if (!FuelTestComplete)
      ISOByte2 = ISOByte2 | 0b00100000;
    if (ComponentsTestAvailable)
      ISOByte2 = ISOByte2 | 0b00000100;
    if (!ComponentsTestComplete)
      ISOByte2 = ISOByte2 | 0b01000000;
    //if (MisfireTestAvailable) BitSet(ISOByte2, 0);
    
    boolean SparkIgnition = true;
    
    if (SparkIgnition)
      ISOByte2 = ISOByte2 | 0b00001000;
      
    
    ISOStructure[0] = ISOByte1;
    ISOStructure[1] = ISOByte2;
    ISOStructure[2] = 0;
    ISOStructure[3] = 0;
    ISO_Measurements_to_ISO_Reply_double_word_bit_encoded_matrix(ISOPID(originalmode, originalpid), ISOStructure);
  }  
  
    void DTCs(ISOPID isopid)
  { 
    GeneralAsyncRequest( isopid, HONDA_RPM.Address, HONDA_RPM.Length, &HondaHandler::Async_Callback_DTCs);
  }
  
  void Async_Callback_DTCs(byte* HondaReadyData)
  {  
      int ISOMeasure = 0;
    
      byte RPM_HI = HondaReadyData[3];
  
      int iDTC0;
      
      int iDTC1;
      int iDTC2;
      
      char cDTC1[2];
      char cDTC2[2];
      
      char DTC[4] = "P06";
      
      if (DTC[0] == 'P')
        iDTC0 = 0;
      if (DTC[0] == 'C')
        iDTC0 = 1;
      if (DTC[0] == 'B')
        iDTC0 = 2;
      if (DTC[0] == 'U')
        iDTC0 = 3;
        
     byte _DTC0_part = iDTC0 << 6;
     
     cDTC1[0] = DTC[1];
     cDTC1[1] = 0;
     
     iDTC1 = strtol( cDTC1, 0, 16 );
     
     byte _DTC1_part = iDTC1 << 4;
     
     cDTC2[0] = DTC[2];
     cDTC2[1] =0;
     
     iDTC2 = strtol ( cDTC2, 0, 16);
     
     byte _DTC2_part = iDTC2;
     
     byte ISOByte = ((0x00000000 | _DTC0_part) | _DTC1_part) | _DTC2_part;
      
    byte DTCMatrix[6];
    
    //int NumberOfDTCs =2 ;
    //byte NumberOfDTCs_part = NumberOfDTCs;// << 4;
    
    //byte SingleFrame = 0b000000000;
    
    //byte ISOFormatByte = 0b00000000 & SingleFrame;
    //ISOFormatByte |= NumberOfDTCs_part;

    DTCMatrix[5] = 0; 
    DTCMatrix[4] = 0;   
    DTCMatrix[3] = 0;
    DTCMatrix[2] = 0;
    DTCMatrix[1] = 0;
    DTCMatrix[0] = ISOByte;
    
    ISO_Measurements_to_ISO_Reply_six_byte_matrix(ISOPID(originalmode, originalpid), DTCMatrix);

  }
  
  
  
  
  
  void GeneralAsyncRequest( ISOPID _ISOPID, char Address, int Length, void (HondaHandler::*__callbackpointer)(byte *) )
  {
    originalmode = _ISOPID.Mode;
    originalpid = _ISOPID.PID;

    callback_pointer = __callbackpointer;
    HondaRequest request( Address, Length);
    HondaRequestSend(request);
    oHondaCommCoord.AssignWork( request );
  }
  
  void HondaRequestSend(  HondaRequest request )
  {   HondaPort.listen();
      HondaPort._reset_c();
      HondaPort._write(0x20); 
      HondaPort._write(0x05);
      HondaPort._write( request.Address );
      HondaPort._write( request.Length );
      HondaPort._compute_send_c();
  }
  
  void IncomingPID( ISOPID _ISOPID )
  {
    //*_OriginalPID = _ISOPID;
    originalmode = _ISOPID.Mode;
    originalpid = _ISOPID.PID;
    
    ISOPID_TO_GENERIC_HANDLER_MAP(_ISOPID); 
  }
};
  
  
  
  HondaHandler oHandler;

  boolean HondaCommMessageCollector::DoneWorking()
  { 
    return (_State == complete);
  }

  void HondaCommMessageCollector::Reset()
  {
    HondaMsgIdx = 0;
    _State = comm_estab;
  }

  void HondaCommMessageProcessor::Reset()
  {
    _State = received;
  }

  boolean HondaCommMessageProcessor::DoneWorking()
  {
    return (_State == processed);
  }

  void HondaCommMessageProcessor::Work(byte *Message)
  {
    if (_State == received)
    {                    
      _State = processed;
      oHandler.Callback(Message);
    }
  }


class TryReady
{
  private :
  enum Shake { no_comm, try_ready, ready };
  long try_ready_tod;
  Shake _State;
  
  public:
  int _BusPin;
  
  TryReady() { try_ready_tod = 0; Reset(); }
  
  boolean DoneWorking()
  { 
    return ( _State == ready );
  }
  
  void Reset()
  {
    _State = no_comm;
    try_ready_tod = 0;
  }
  
  void Work()
  {
    if (_State == no_comm)
    { 
      if (digitalRead( _BusPin ) == HIGH)
      {
        _State = try_ready;
        try_ready_tod = millis();  
      }
    }
    
    if (_State == try_ready)
    {
      if (  (millis() - this->try_ready_tod) > 100   )
      {
        _State = ready;
      }
      else
      if (digitalRead( _BusPin ) == LOW)
      {
        _State = no_comm;
      }
    }
  }
};
      
class GenericBusInitComplete
{
  private:
  protected:
  enum Shake { attempt_confirm_sync, attempt_confirm_sync_collecting, sync_confirmed };
  protected:
  Shake _State;

  int sync_collecting_timeout;
  
  public:
  virtual void ProtocolWork()=0;
  
  void Work()
  {
    if (_State == attempt_confirm_sync || _State == attempt_confirm_sync_collecting )
    {
      ProtocolWork();
    }
  }
  
  void Reset()
  {
    _State = attempt_confirm_sync;
  }

  boolean DoneWorking()
  {
    return (_State == sync_confirmed);
  } 
};

class BusInitComplete_ISO14230_SLOW : public GenericBusInitComplete
{
  void ProtocolWork()
  {
        byte c = 0;
        if (ISOPort.available())
        {
           c = ISOPort.read();
           if (c == 0x70)
           {
             ISOPort.write(0xCC);
             _State = sync_confirmed;
           }
        }
      }
};


class BusInitComplete_ISO9141 : public GenericBusInitComplete
{
  void ProtocolWork()
  {
    byte c = 0;
        if (ISOPort.available())
        {
           c = ISOPort.read();
           if (c == 0x6B)
           {
             ISOPort.write(0xCC);
             _State = sync_confirmed;
           }
        }
  }
};

class BusInitComplete_ISO14230_FAST : public GenericBusInitComplete
{
    private:
      byte SYNC_MSG[8];
      int sync_msg_idx;
    public:
  
    void ProtocolWork()
    { 
      if (_State == attempt_confirm_sync   ||  _State == attempt_confirm_sync_collecting)
          { 
            byte b = 0;

            if (_State == attempt_confirm_sync)
            { 
              sync_msg_idx = 0; 
              ISOPort._begin();
              _State = attempt_confirm_sync_collecting;
            }
            
            if (ISOPort.available())
            { 
               b = ISOPort.read();  
               SYNC_MSG[sync_msg_idx] = b;
               sync_msg_idx++;
            }
    
    if (  sync_msg_idx == 5 && 
          SYNC_MSG[0] == 0xC1 &&
          SYNC_MSG[1] == 0x33 &&
          SYNC_MSG[2] == 0xF1 &&
          SYNC_MSG[3] == 0x81 &&
          SYNC_MSG[4] == 0x66)
          {
            ISOPort._reset_c();
            ISOPort._write(0x83);
            delay(5);
            ISOPort._write(0xF1);
            delay(5);
            ISOPort._write(0x40);
            delay(5);
            ISOPort._write(0xC1);
            delay(5);
            ISOPort._write(0xE9);
            delay(5);
            ISOPort._write(0x8F);
            delay(5);
            ISOPort._compute_send_c();
            _State = sync_confirmed;
          }
          }
    }
};
      

class GenericBusInitAttempt
{
  private:
  boolean ATTEMPT_BUS_INIT_LAST_PIN_STATE;
  long ATTEMPT_BUS_INIT_START_TIME;
  long ATTEMPT_BUS_INIT_LAST_TRANSITION_TOD;
  
  protected:
  int WAKE_PACK[10];
  int WAKE_PACK_IDX ;//ajs= 0;

  enum Shake { bus_ready, attempt_bus_init, sync_attempt };
  Shake _State;
  
  boolean valid_wake_pack_time(int pack_time)
   {
     return ( pack_time > 375 && pack_time < 425 );
   }
 
   boolean valid_fast_init_wake_pack_time(int pack_time)
   {
     return ( pack_time > 20 && pack_time < 30 );
   }
 
  public : 
  void Reset()
  {
    _State = bus_ready;
    WAKE_PACK_IDX = 0;
  }
  
  boolean DoneWorking()
  {
    return ( _State == sync_attempt );
  }
  
  virtual void ProtocolWork()=0;
  
  void Work()
  {
    if (_State == bus_ready)
    { 
      if (digitalRead(6)==LOW) 
      {
        ATTEMPT_BUS_INIT_LAST_PIN_STATE = LOW;
        ATTEMPT_BUS_INIT_START_TIME = millis();
        ATTEMPT_BUS_INIT_LAST_TRANSITION_TOD = millis();
        WAKE_PACK_IDX = 0;
        _State = attempt_bus_init;
      }
    }
    
    if (_State == attempt_bus_init)
    {
      if (digitalRead(6) != ATTEMPT_BUS_INIT_LAST_PIN_STATE)
      {
        WAKE_PACK[WAKE_PACK_IDX] = millis() - ATTEMPT_BUS_INIT_LAST_TRANSITION_TOD;
        ATTEMPT_BUS_INIT_LAST_TRANSITION_TOD = millis();
        ATTEMPT_BUS_INIT_LAST_PIN_STATE = !ATTEMPT_BUS_INIT_LAST_PIN_STATE;
        WAKE_PACK_IDX++;
        for (int i=0; i<WAKE_PACK_IDX; i++) {Serial.println("wake:"); Serial.println(WAKE_PACK[i]); }Serial.println();
      }
      
      //if (WAKE_PACK_IDX > 9)
      //  { SHAKE_LEVEL = SHAKE_NO_COMM;  }
      
      if (  ( millis() - ATTEMPT_BUS_INIT_START_TIME ) > 2000  )
      {
        this -> Reset(); Serial.println("slow init timeout");
      }

      ProtocolWork();
   }
  }
};


class BusInitAttempt_ISO14230_SLOW : public GenericBusInitAttempt
{
  void ProtocolWork()
  {
    if (WAKE_PACK_IDX == 5
        && valid_wake_pack_time(WAKE_PACK[1])
        && valid_wake_pack_time(WAKE_PACK[2])
        && valid_wake_pack_time(WAKE_PACK[3])
        )
        {
            delay(200);
            ISOPort._begin(); 
            delay(15);
            ISOPort._write(0x55);
            delay(40);
            ISOPort._write(0xE9);
            delay(5);
            ISOPort._write(0x8F);
            _State = sync_attempt;
        }
  }
};

class BusInitAttempt_ISO14230_FAST : public GenericBusInitAttempt
{
  void ProtocolWork()
  {
    if (WAKE_PACK_IDX == 1 && valid_fast_init_wake_pack_time(WAKE_PACK[0]) )
      {
        _State = sync_attempt;
      }
  }
};
    
    
    
class BusInitAttempt_ISO9141 : public GenericBusInitAttempt
{
  void ProtocolWork()
  {
      if (WAKE_PACK_IDX == 5
        && valid_wake_pack_time(WAKE_PACK[1])
        && valid_wake_pack_time(WAKE_PACK[2])
        && valid_wake_pack_time(WAKE_PACK[3])
        )
        {
            delay(200);
            ISOPort._begin(); 
            delay(15);
            ISOPort._write(0x55);
            delay(40);
            ISOPort._write(0x94);
            delay(5);
            ISOPort._write(0x94);
            _State = sync_attempt;
        }
  }
};


BusInitAttempt_ISO9141 oBusInitAttempt_ISO9141;
BusInitAttempt_ISO14230_FAST oBusInitAttempt_ISO14230_FAST;
BusInitAttempt_ISO14230_SLOW oBusInitAttempt_ISO14230_SLOW;
BusInitComplete_ISO9141 oBusInitComplete_ISO9141;
BusInitComplete_ISO14230_FAST oBusInitComplete_ISO14230_FAST;
BusInitComplete_ISO14230_SLOW oBusInitComplete_ISO14230_SLOW;

long sync_sent_tod;

class BusInit
{
  private:
  GenericBusInitAttempt* oBusInitAttempt;
  GenericBusInitComplete* oBusInitComplete;

  enum Shake { bus_ready, sync_attempt, sync_complete };
  Shake _State;
  
  public: 
  BusInit()
  {
      if (Protocol == ISO9141)
      {
        oBusInitAttempt = &oBusInitAttempt_ISO9141;
        oBusInitComplete = &oBusInitComplete_ISO9141;
      }
      else
      if (Protocol == ISO14230_Slow_Init)
      {
        oBusInitAttempt = &oBusInitAttempt_ISO14230_SLOW;
        oBusInitComplete = &oBusInitComplete_ISO14230_SLOW;
      }
      else
      if (Protocol == ISO14230_Fast_Init)
      {
        oBusInitAttempt = &oBusInitAttempt_ISO14230_FAST;
        oBusInitComplete = &oBusInitComplete_ISO14230_FAST;
      }
  }
  
  void Reset()
  {
    _State = bus_ready;
    oBusInitAttempt->Reset();
    oBusInitComplete->Reset();
  }
  
  boolean DoneWorking()
  {
    return ( _State == sync_complete );
  }
  
  void Work()
  {
    if ( _State == bus_ready )
    {
      oBusInitAttempt->Work();
      if (oBusInitAttempt->DoneWorking())
      {
        _State = sync_attempt;
        sync_sent_tod=millis();
      }
    }
    if ( _State == sync_attempt )
    {
      if ((millis() - sync_sent_tod) > 2000)
      {
        ISOPort.end();
        this -> Reset();
        Serial.println("sync timeout");
      }

      oBusInitComplete->Work();
      if (oBusInitComplete->DoneWorking())
          _State = sync_complete;
    }
  }
};


class BusCoordinator
{
  private:
    
  TryReady oTryReady;
  BusInit oBusInit;
  enum Shake { no_comm, ready, comm_estab };
  Shake _State;
  public:
  
  BusCoordinator( int BusPin )
  {
    oTryReady._BusPin = BusPin;
  }
  
  int _BusPin;
  
  void Reset()
  {
    oTryReady.Reset();
    oBusInit.Reset();
    _State = no_comm;
  }
  
  void Work()
  { 
    if (_State == no_comm)
    {
      oTryReady.Work();
      if ( oTryReady.DoneWorking() )
      {
        _State = ready;
        oBusInit.Reset();
      }
    }
    
    if (_State == ready)
    {
      oBusInit.Work();
      if ( oBusInit.DoneWorking() )
          _State = comm_estab;
    }
        
  };
  boolean DoneWorking()
  {
    return ( _State == comm_estab );
  }
  
};


class GenericCommMessageCollector
{
  
  protected:
  int scan_msg_idx;
  
  private:
  enum Shake { comm_estab, collect, complete };
  Shake _State;
  boolean burst;
  int burst_limit;
  long start_collecting_tod;
  int timeouts;
  
  private:
  
  void _Reset()
  {
    _State = comm_estab;
    burst = false;
    burst_limit = 0;
    start_collecting_tod = 0;
  }
  
  public:
  
  virtual int ExpectedLen(byte* SCAN_MSG)=0;
  
  int Timeouts()
  { 
    return timeouts;
  }
  
  void Reset()
  {
    timeouts = 0;
    this->_Reset();
  }
  
  boolean DoneWorking()
  {
    return (_State == complete);
  }
  
  void Work(byte* SCAN_MSG)
  {
    if (_State == comm_estab || _State == collect )
    { 
      byte b = 0;
      
      if (_State == comm_estab)
      {
        scan_msg_idx = 0;
        start_collecting_tod = millis();
        ISOPort._reset_c();
        burst = false;
      }

      if (_State == collect)
      {
        if (  (millis() - start_collecting_tod) > 200)
        { 
          timeouts++;
          this -> _Reset();
          Serial.println("collecting message timeout");
        }
      }
    
      do { 
              if (ISOPort.available())
              {
                 b = ISOPort._read();
                 SCAN_MSG[scan_msg_idx] = b;
                 scan_msg_idx++;
                 _State = collect;
                 if (!burst)
                 {
                   burst = true;
                   burst_limit = 0;
                 }
              }
    
              if (_State == collect)
              {
                burst_limit++;

                if (  (scan_msg_idx == ExpectedLen(SCAN_MSG) ) && ISOPort._valid_checksum( b )  )
                {
                  burst = false;
                  _State = complete;
                }
              }
        } while (burst && (burst_limit < SERIAL_BURST_MAX_TIME_SLICE));
        burst=false;
    
    }
  }
};


class ISOCommMessageCollector_ISO9141 : public GenericCommMessageCollector
{
  int ExpectedLen(byte* SCAN_MSG)
  {
    return 6;
  }
};

class ISOCommMessageCollector_ISO14230 : public GenericCommMessageCollector
{ 
  
  int ExpectedLen(byte* SCAN_MSG)
  {
    int _expected_len = 0;
    if (scan_msg_idx >= 1)
       _expected_len =  (  ( SCAN_MSG[0] & 0b00111111 ) + 4);
    return _expected_len;
  }
};


class GenericCommMessageProcessor
{
  private:
  enum Shake { received, processed, ignored };
  Shake _State;
  
  public:

  IHandler* oHandler;
  
  public:
  virtual boolean ValidHeaders( byte* SCAN_MSG )=0;
  
  void Reset()
  {
    _State = received;
  }
  
  boolean DoneWorking()
  {
    return (  (_State == processed || _State == ignored) );
  }
  
  boolean ValidMessage()
  {
    return (  _State == processed );
  }
  
  void Work(byte* SCAN_MSG)
  {
    if (_State == received)
    {
      Serial.println("Received: ");
      
      for ( int MsgCharIndex = 0; MsgCharIndex< 7; MsgCharIndex++)
      { Serial.print(SCAN_MSG[MsgCharIndex], HEX);
       Serial.print(",");
      }

      byte SCANNER_COMMAND_BYTE;
      byte SCANNER_MODE;
      
      if ( ValidHeaders( SCAN_MSG ) )
          {
            SCANNER_COMMAND_BYTE = SCAN_MSG[4];
            SCANNER_MODE = SCAN_MSG[3];
            _State = processed;
            oHandler->IncomingPID( ISOPID(SCANNER_MODE, SCANNER_COMMAND_BYTE) ); Serial.println("iso messaging pid");
           //else
           //{
             //Serial.println("unknown command or PID");
             //_State = ignored;
           //}  
          }
          else
          {
            _State = ignored;
            Serial.println("faulty headers");
          }
    }
  }
};

class ISOCommMessageProcessor_ISO9141 :  public GenericCommMessageProcessor
{
  boolean ValidHeaders( byte* SCAN_MSG )
  {
    return ISO9141_Message_Headers_OK( SCAN_MSG );
  };
};

class ISOCommMessageProcessor_ISO14230 : public GenericCommMessageProcessor
{
  boolean ValidHeaders ( byte* SCAN_MSG )
  {
    return KWP_Message_Headers_OK( SCAN_MSG );
  }
};


ISOCommMessageProcessor_ISO9141 oISOCommMessageProcessor_ISO9141;
ISOCommMessageProcessor_ISO14230 oISOCommMessageProcessor_ISO14230;

ISOCommMessageCollector_ISO9141 oISOCommMessageCollector_ISO9141;
ISOCommMessageCollector_ISO14230 oISOCommMessageCollector_ISO14230;

long comm_estab_tod;

class CommCoordinator
{
  private:
  enum Shake { comm_estab, message_received, message_processed };
  Shake _State;
  GenericCommMessageCollector* oCommMessageCollector;
  
  public:
  GenericCommMessageProcessor* oCommMessageProcessor;
  
  protected:
    byte SCAN_MSG[30];
  
  public: 
  CommCoordinator(IHandler* z)
  {
    if (Protocol == ISO9141)
    {
      oCommMessageCollector = &oISOCommMessageCollector_ISO9141;
      oCommMessageProcessor = &oISOCommMessageProcessor_ISO9141;
    }
    else
    {
      oCommMessageCollector = &oISOCommMessageCollector_ISO14230;
      oCommMessageProcessor = &oISOCommMessageProcessor_ISO14230;
    }
    oCommMessageProcessor->oHandler = z;
  }
  
  boolean DoneWorking()
  {
    return (_State == message_processed);
  }
  
  void Reset()
  {
    _State = comm_estab;
    oCommMessageCollector->Reset();
    oCommMessageProcessor->Reset();
  }
  
  void Work()
  {
    if (_State == comm_estab)
    {
      oCommMessageCollector->Work( SCAN_MSG );
      if (oCommMessageCollector->DoneWorking())
      {
         _State = message_received;
      }
      if (oCommMessageCollector->Timeouts() > 2)
      {
        Serial.println("communications reset (excessive timeouts)");
        this->Reset();
      }
    }
    if (_State == message_received)
    {
      oCommMessageProcessor->Work( SCAN_MSG );
      if (oCommMessageProcessor->DoneWorking())
      {
        if (oCommMessageProcessor->ValidMessage())
        {
          comm_estab_tod = millis();
        }
        _State = message_processed;
        oCommMessageCollector->Reset();
      }
    }
    
    if (_State == message_processed)
    {
      _State = comm_estab;
      oCommMessageProcessor->Reset();
    }
  }

}
;


BusCoordinator oBusCoord ( 6 );

CommCoordinator oCommCoord ( &oHandler );


void setup()  
{
  Serial.begin(9600);
  
  pinMode(6,INPUT);
  
  HondaPort._begin();
  
  simcount=0;ramp = 0;
  pinMode(5,OUTPUT);
  analogWrite(5,100);

  oBusCoord.Reset();
  
  oCommCoord.Reset();
  //oCommCoord.oCommMessageProcessor.oMessageHandler = oHandler;
  
  oHondaCommCoord.Reset();
  
}

 
void loop() 
{

    simcount++;
    
    if (  (simcount % 500) == 0)
      {
        ramp++;
        analogWrite(5,ramp);
        if (ramp == 25) ramp = 0;
      }

    if (SHAKE_LEVEL == SHAKE_NO_COMM)
    {
      oBusCoord.Work();
    
      if (oBusCoord.DoneWorking())
      {
        comm_estab_tod = millis();
        SHAKE_LEVEL = SHAKE_COMM_ESTAB;
      }
    }
    
    if (SHAKE_LEVEL == SHAKE_COMM_ESTAB)
    {
      oCommCoord.Work();
      if (oCommCoord.DoneWorking())
      {
        //??
      }
    }
    
    oHondaCommCoord.Work();
    

  if (SHAKE_LEVEL == SHAKE_COMM_ESTAB)
  { 
      if (  (millis() - comm_estab_tod) > 5000)
      { 
        SHAKE_LEVEL = SHAKE_NO_COMM;
        oBusCoord.Reset();
        oCommCoord.Reset();
        ISOPort.end(); 
        Serial.println("comm timeout");
      }
  }
  
}


boolean ISO9141_Message_Headers_OK( byte* _SCAN_MSG )
{
  boolean ok = false;
  
  if (
        _SCAN_MSG[0] == SAE_J1979_HEAD &&
        _SCAN_MSG[1] == OBDII_HEAD  &&
        _SCAN_MSG[2] == SCAN_HEAD &&
        ( valid_mode(_SCAN_MSG[3]) || keep_alive(_SCAN_MSG[3]) ) 
     ) 
        {
          ok = true;
        }
        
        return ok;
}


boolean KWP_Message_Headers_OK( byte* _SCAN_MSG )
{
  boolean ok = false;
  
  if (
        (  (_SCAN_MSG[0] & 0b11000000) == 0b11000000   ) &&
         _SCAN_MSG[1] == KWP_SLOW_INIT_HEAD &&
        _SCAN_MSG[2] == SCAN_HEAD &&
        ( valid_mode(_SCAN_MSG[3]) || keep_alive(_SCAN_MSG[3]) ) 
     ) 
        {
          ok = true;
        }
        
        return ok;
}


