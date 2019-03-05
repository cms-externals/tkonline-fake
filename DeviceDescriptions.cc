#include <vector>
#include <map>
#include <list>
#include <stdexcept>
#include <string>
#include <stdint.h>

#define THROW_RUNTIME                                        \
  throw std::runtime_error(                                  \
      "Unsupported call to trackerDAQ library, function: " + \
      std::string(__PRETTY_FUNCTION__))

class ConnectionDescription;
class deviceDescription;
class TkDcuInfo;

class TkState {};
class TkRun {};

enum enumDeviceType {
  PLL,
  LASERDRIVER,
  DOH,
  DCU,
  PHILIPS,
  APVMUX,
  APV25,
  FOREXTENDED,
  PIARESET,
  FECMEMORY,
  DELTA,
  PACE,
  KCHIP,
  GOH,
  VFAT,
  CCHIP,
  ESMBRESET,
  TBB,
  DELAY25
};

typedef std::vector<ConnectionDescription *> ConnectionVector;
typedef std::vector<deviceDescription *> deviceVector;
typedef std::map<uint32_t, std::pair<uint32_t, uint32_t> >
    HashMapAnalysisVersions;
typedef std::map<uint32_t, std::vector<std::pair<uint32_t, uint32_t> > >
    HashMapRunVersion;
typedef std::vector<TkState *> tkStateVector;
typedef std::vector<TkRun *> tkRunVector;
typedef uint16_t tscType16;
typedef uint32_t tscType32;
typedef uint8_t tscType8;
typedef std::vector<uint16_t> VInt16;

namespace Fed9U {
class Fed9UDescription {};
}

#ifdef __GNUC__
#include <ext/hash_map>
namespace Sgi = ::__gnu_cxx;
#endif

class CommissioningAnalysisDescription {
 public:
  // TOOD: Not needed this one? Delete then,
  enum {
    CRATE,
    SLOT,
    RING,
    CCUADR,
    CCUCHAN,
    I2CADDR,
    PARTITION,
    RUNNUMBER,
    ANALYSISTYPE,
    VALID,
    COMMENTS,
    FEDID,
    FEUNIT,
    FECHAN,
    FEDAPV,
    DCUHARDID,
    DETID,
    ENDOFNAMEENUM
  };

  typedef enum {
    T_UNKNOWN,
    T_ANALYSIS_APVLATENCY,
    T_ANALYSIS_CALIBRATION,
    T_ANALYSIS_FASTFEDCABLING,
    T_ANALYSIS_FINEDELAY,
    T_ANALYSIS_OPTOSCAN,
    T_ANALYSIS_PEDESTALS,
    T_ANALYSIS_TIMING,
    T_ANALYSIS_VPSPSCAN,
    T_ANALYSIS_PEDSFULLNOISE
  } commissioningType;
  uint16_t getCrate() const;
  uint16_t getSlot() const;
  uint16_t getRing() const;
  uint16_t getCcuAdr() const;
  uint16_t getCcuChan() const;
  uint16_t getI2cAddr() const;
  void addComments(const std::string value);
};

uint16_t CommissioningAnalysisDescription::getCrate() const { THROW_RUNTIME; };
uint16_t CommissioningAnalysisDescription::getSlot() const { THROW_RUNTIME; };
uint16_t CommissioningAnalysisDescription::getRing() const { THROW_RUNTIME; };
uint16_t CommissioningAnalysisDescription::getCcuAdr() const { THROW_RUNTIME; };
uint16_t CommissioningAnalysisDescription::getCcuChan() const {
  THROW_RUNTIME;
};
uint16_t CommissioningAnalysisDescription::getI2cAddr() const {
  THROW_RUNTIME;
};
void CommissioningAnalysisDescription::addComments(const std::string value) {
  THROW_RUNTIME;
};

// class DeviceFactoryInterface { };

// class CommissioningAnalysisFactory : public DeviceFactoryInterface {
class CommissioningAnalysisFactory {
 public:
  uint32_t uploadAnalysis(
      uint32_t runNumber, std::string partitionName,
      CommissioningAnalysisDescription::commissioningType type,
      std::vector<CommissioningAnalysisDescription *> &descriptions,
      bool updateCurrentState);
  void uploadAnalysisState(uint32_t uploadedVersion);
  HashMapRunVersion getAnalysisHistory(
      std::string partitionName,
      CommissioningAnalysisDescription::commissioningType type);
  std::vector<CommissioningAnalysisDescription *> getAnalysisHistory(
      std::string partitionName, uint32_t versionMajorID,
      uint32_t versionMinorID,
      CommissioningAnalysisDescription::commissioningType type);
  HashMapAnalysisVersions getLocalAnalysisVersions(
      uint32_t globalAnalysisVersion);
};

uint32_t CommissioningAnalysisFactory::uploadAnalysis(
    uint32_t runNumber, std::string partitionName,
    CommissioningAnalysisDescription::commissioningType type,
    std::vector<CommissioningAnalysisDescription *> &descriptions,
    bool updateCurrentState) {
  THROW_RUNTIME;
};
void CommissioningAnalysisFactory::uploadAnalysisState(
    uint32_t uploadedVersion) {
  THROW_RUNTIME;
};
HashMapRunVersion CommissioningAnalysisFactory::getAnalysisHistory(
    std::string partitionName,
    CommissioningAnalysisDescription::commissioningType type) {
  THROW_RUNTIME;
};
std::vector<CommissioningAnalysisDescription *>
CommissioningAnalysisFactory::getAnalysisHistory(
    std::string partitionName, uint32_t versionMajorID, uint32_t versionMinorID,
    CommissioningAnalysisDescription::commissioningType type) {
  THROW_RUNTIME;
};
HashMapAnalysisVersions CommissioningAnalysisFactory::getLocalAnalysisVersions(
    uint32_t globalAnalysisVersion) {
  THROW_RUNTIME;
};

class ConnectionDescription {
 public:
  ConnectionDescription(unsigned int fedId = 0, unsigned int fedChannel = 0,
                        std::string fecHardwareId = "",
                        unsigned int fecCrateId = 0, unsigned int fecSlot = 0,
                        unsigned int ringSlot = 0, unsigned int ccuAddress = 0,
                        unsigned int i2cChannel = 0,
                        unsigned int apvAddress = 0, unsigned int dcuHardId = 0,
                        bool enabled = true, unsigned int fedCrateId = 0,
                        unsigned int fedSlot = 0);
  ConnectionDescription *clone();
};

ConnectionDescription::ConnectionDescription(
    unsigned int fedId, unsigned int fedChannel, std::string fecHardwareId,
    unsigned int fecCrateId, unsigned int fecSlot, unsigned int ringSlot,
    unsigned int ccuAddress, unsigned int i2cChannel, unsigned int apvAddress,
    unsigned int dcuHardId, bool enabled, unsigned int fedCrateId,
    unsigned int fedSlot) {
  THROW_RUNTIME;
};
ConnectionDescription *ConnectionDescription::clone() { THROW_RUNTIME; };

// class ConnectionFactory: public DeviceFactoryInterface {
class ConnectionFactory {
 public:
  void getConnectionDescriptions(
      std::string partitionName, ConnectionVector &outVector,
      unsigned int versionMajor = 0, unsigned int versionMinor = 0,
      unsigned int maskVersionMajor = 0, unsigned int maskVersionMinor = 0,
      bool allConnections = false, bool forceDbReload = false);
  void setConnectionDescriptions(ConnectionVector connectionVector,
                                 std::string partitionName = "nil",
                                 unsigned int *versionMajor = NULL,
                                 unsigned int *versionMinor = NULL,
                                 bool majorVersion = false);
};

void ConnectionFactory::getConnectionDescriptions(
    std::string partitionName, ConnectionVector &outVector,
    unsigned int versionMajor, unsigned int versionMinor,
    unsigned int maskVersionMajor, unsigned int maskVersionMinor,
    bool allConnections, bool forceDbReload) {
  THROW_RUNTIME;
};
void ConnectionFactory::setConnectionDescriptions(
    ConnectionVector connectionVector, std::string partitionName,
    unsigned int *versionMajor, unsigned int *versionMinor, bool majorVersion) {
  THROW_RUNTIME;
};

class DbClient {
 public:
  DbClient(std::string name);
  ~DbClient();
  void parse();
  deviceVector *getDevices();
  std::vector<Fed9U::Fed9UDescription *> *getFed9UDescriptions();
  ConnectionVector *getConnections();
  Sgi::hash_map<unsigned long, TkDcuInfo *> *getInfos();
};

DbClient::DbClient(std::string name) { THROW_RUNTIME; };
DbClient::~DbClient() { THROW_RUNTIME; };
void DbClient::parse() { THROW_RUNTIME; };
deviceVector *DbClient::getDevices() { THROW_RUNTIME; };
std::vector<Fed9U::Fed9UDescription *> *DbClient::getFed9UDescriptions() {
  THROW_RUNTIME;
};
ConnectionVector *DbClient::getConnections() { THROW_RUNTIME; };
Sgi::hash_map<unsigned long, TkDcuInfo *> *DbClient::getInfos() {
  THROW_RUNTIME;
};

class deviceDescription {
 public:
  bool isEnabled();
  enumDeviceType getDeviceType();
  tscType16 getCrateId();
  tscType16 getFecSlot();
  tscType16 getRingSlot();
  tscType16 getCcuAddress();
  tscType16 getChannel();
  tscType16 getAddress();
  virtual ~deviceDescription();
};

bool deviceDescription::isEnabled() { THROW_RUNTIME; };
enumDeviceType deviceDescription::getDeviceType() { THROW_RUNTIME; };
tscType16 deviceDescription::getCrateId() { THROW_RUNTIME; };
tscType16 deviceDescription::getFecSlot() { THROW_RUNTIME; };
tscType16 deviceDescription::getRingSlot() { THROW_RUNTIME; };
tscType16 deviceDescription::getCcuAddress() { THROW_RUNTIME; };
tscType16 deviceDescription::getChannel() { THROW_RUNTIME; };
tscType16 deviceDescription::getAddress() { THROW_RUNTIME; };
deviceDescription::~deviceDescription() { THROW_RUNTIME; };

// class DeviceFactory: public FecFactory,
// public Fed9U::Fed9UDeviceFactory,
// public ConnectionFactory,
// public TkDcuConversionFactory,
// public TkDcuInfoFactory,
// public TkDcuPsuMapFactory,
// public TkIdVsHostnameFactory,
// public CommissioningAnalysisFactory,
// public TkMaskModulesFactory
class DeviceFactory {
 public:
  DeviceFactory();
  DeviceFactory(std::string login, std::string password, std::string path,
                bool threaded = false);
  void addFecFileName(std::string fileName);
  void addFedFileName(std::string fileName);
  void addConnectionFileName(std::string fileName);
  void addTkDcuInfoFileName(std::string fileName);
  void addTkIdVsHostnameFileName(std::string fileName);
  void setUsingDb(bool useIt = true);
};

DeviceFactory::DeviceFactory() { THROW_RUNTIME; };
DeviceFactory::DeviceFactory(std::string login, std::string password,
                             std::string path, bool threaded) {
  THROW_RUNTIME;
};
void DeviceFactory::addFecFileName(std::string fileName) { THROW_RUNTIME; };
void DeviceFactory::addFedFileName(std::string fileName) { THROW_RUNTIME; };
void DeviceFactory::addConnectionFileName(std::string fileName) {
  THROW_RUNTIME;
};
void DeviceFactory::addTkDcuInfoFileName(std::string fileName) {
  THROW_RUNTIME;
};
void DeviceFactory::addTkIdVsHostnameFileName(std::string fileName) {
  THROW_RUNTIME;
};
void DeviceFactory::setUsingDb(bool useIt) { THROW_RUNTIME; };

// class FecDeviceFactory: public DeviceFactoryInterface {
class FecDeviceFactory {
 public:
  void getFecDeviceDescriptions(
      bool fileUsed, unsigned int versionMajor, unsigned int versionMinor,
      unsigned int pMaskVersionMajorId, unsigned int pMaskVersionMinorId,
      std::string partitionName, std::string fecHardwareId,
      deviceVector &outVector, bool allDevices = false,
      bool forceDbReload = false);
  void getFecDeviceDescriptions(std::string partitionName,
                                deviceVector &outVector,
                                unsigned int versionMajor = 0,
                                unsigned int versionMinor = 0,
                                unsigned int pMaskVersionMajorId = 0,
                                unsigned int pMaskVersionMinorId = 0,
                                bool allDevices = false,
                                bool forceDbReload = false);
  void setFecDeviceDescriptions(deviceVector devices,
                                std::string partitionName = "nil",
                                unsigned int *versionMajor = NULL,
                                unsigned int *versionMinor = NULL,
                                bool majorVersion = false,
                                bool uploadVersion = false);
};

void FecDeviceFactory::getFecDeviceDescriptions(
    bool fileUsed, unsigned int versionMajor, unsigned int versionMinor,
    unsigned int pMaskVersionMajorId, unsigned int pMaskVersionMinorId,
    std::string partitionName, std::string fecHardwareId,
    deviceVector &outVector, bool allDevices, bool forceDbReload) {
  THROW_RUNTIME;
};
void FecDeviceFactory::getFecDeviceDescriptions(
    std::string partitionName, deviceVector &outVector,
    unsigned int versionMajor, unsigned int versionMinor,
    unsigned int pMaskVersionMajorId, unsigned int pMaskVersionMinorId,
    bool allDevices, bool forceDbReload) {
  THROW_RUNTIME;
};
void FecDeviceFactory::setFecDeviceDescriptions(
    deviceVector devices, std::string partitionName, unsigned int *versionMajor,
    unsigned int *versionMinor, bool majorVersion, bool uploadVersion) {
  THROW_RUNTIME;
};

// class TkDcuInfoFactory: public DeviceFactoryInterface {
class TkDcuInfoFactory {
 public:
  void addDetIdPartition(std::string partitionName,
                         unsigned int majorVersionId = 0,
                         unsigned int minorVersionId = 0,
                         bool cleanCache = true, bool forceDbReload = false);
  tkStateVector &getCurrentStates();
  TkRun *getLastRun(std::string partitionName);
  TkRun *getRun(std::string partitionName, int runNumber);
  tkRunVector getAllRuns();
  std::list<std::string> getAllPartitionNames(unsigned int runNumber);
  std::list<std::string> getAllPartitionNames();
};

void TkDcuInfoFactory::addDetIdPartition(std::string partitionName,
                                         unsigned int majorVersionId,
                                         unsigned int minorVersionId,
                                         bool cleanCache, bool forceDbReload) {
  THROW_RUNTIME;
};
tkStateVector &TkDcuInfoFactory::getCurrentStates() { THROW_RUNTIME; };
TkRun *TkDcuInfoFactory::getLastRun(std::string partitionName) {
  THROW_RUNTIME;
};
TkRun *TkDcuInfoFactory::getRun(std::string partitionName, int runNumber) {
  THROW_RUNTIME;
};
tkRunVector TkDcuInfoFactory::getAllRuns() { THROW_RUNTIME; };
std::list<std::string> TkDcuInfoFactory::getAllPartitionNames(
    unsigned int runNumber) {
  THROW_RUNTIME;
};
std::list<std::string> TkDcuInfoFactory::getAllPartitionNames() {
  THROW_RUNTIME;
};

class TkDcuInfo {
 public:
  ~TkDcuInfo();
  tscType32 getDcuHardId();
  tscType32 getDetId();
  double getFibreLength();
  tscType32 getApvNumber();
};

TkDcuInfo::~TkDcuInfo() { THROW_RUNTIME; };
tscType32 TkDcuInfo::getDcuHardId() { THROW_RUNTIME; };
tscType32 TkDcuInfo::getDetId() { THROW_RUNTIME; };
double TkDcuInfo::getFibreLength() { THROW_RUNTIME; };
tscType32 TkDcuInfo::getApvNumber() { THROW_RUNTIME; };

class apvDescription {
 public:
  tscType8 getApvMode();
  tscType8 getLatency();
  tscType8 getVpsp();
  tscType8 getIsha();
  tscType8 getVfs();
  void setIsha(tscType8 isha);
  void setLatency(tscType8 latency);
  void setVfs(tscType8 vfs);
  void setVpsp(tscType8 vpsp);
  virtual ~apvDescription();
};

tscType8 apvDescription::getApvMode() { THROW_RUNTIME; };
tscType8 apvDescription::getLatency() { THROW_RUNTIME; };
tscType8 apvDescription::getVpsp() { THROW_RUNTIME; };
tscType8 apvDescription::getIsha() { THROW_RUNTIME; };
tscType8 apvDescription::getVfs() { THROW_RUNTIME; };
void apvDescription::setIsha(tscType8 isha) { THROW_RUNTIME; };
void apvDescription::setLatency(tscType8 latency) { THROW_RUNTIME; };
void apvDescription::setVfs(tscType8 vfs) { THROW_RUNTIME; };
void apvDescription::setVpsp(tscType8 vpsp) { THROW_RUNTIME; };
apvDescription::~apvDescription() { THROW_RUNTIME; };

class dcuDescription {
  tscType32 getDcuHardId();
  std::string getDcuType();
  virtual ~dcuDescription();
};

tscType32 dcuDescription::getDcuHardId() { THROW_RUNTIME; };
std::string dcuDescription::getDcuType() { THROW_RUNTIME; };
dcuDescription::~dcuDescription() { THROW_RUNTIME; };

class TimingAnalysisDescription {
 public:
  TimingAnalysisDescription(float timetmre, float refTime, float delay,
                            float height, float base, float peak,
                            float frameFindingThreshold,
                            float optimumSamplingPoint,
                            float tickMarkHeightThreshold, bool kind,
                            uint16_t crate, uint16_t slot, uint16_t ring,
                            uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr,
                            std::string partition, uint32_t runNumber,
                            bool valid, std::string comments, uint16_t fedId,
                            uint16_t feUnit, uint16_t feChan, uint16_t fedApv);
  float getHeight() const;
  virtual ~TimingAnalysisDescription();
};

TimingAnalysisDescription::TimingAnalysisDescription(
    float timetmre, float refTime, float delay, float height, float base,
    float peak, float frameFindingThreshold, float optimumSamplingPoint,
    float tickMarkHeightThreshold, bool kind, uint16_t crate, uint16_t slot,
    uint16_t ring, uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr,
    std::string partition, uint32_t runNumber, bool valid, std::string comments,
    uint16_t fedId, uint16_t feUnit, uint16_t feChan, uint16_t fedApv) {
  THROW_RUNTIME;
};
float TimingAnalysisDescription::getHeight() const { THROW_RUNTIME; };
TimingAnalysisDescription::~TimingAnalysisDescription() { THROW_RUNTIME; };

class TkDcuPsuMap {
 public:
  tscType32 getDcuHardId();
};

tscType32 TkDcuPsuMap::getDcuHardId() { THROW_RUNTIME; };

// class ApvLatencyAnalysisDescription : public CommissioningAnalysisDescription
// {
class ApvLatencyAnalysisDescription {
 public:
  ApvLatencyAnalysisDescription(uint16_t latency, uint16_t crate, uint16_t slot,
                                uint16_t ring, uint16_t ccuAdr,
                                uint16_t ccuChan, uint16_t i2cAddr,
                                std::string partition, uint32_t runNumber,
                                bool valid, std::string comments,
                                uint16_t fedId, uint16_t feUnit,
                                uint16_t feChan, uint16_t fedApv);
};

ApvLatencyAnalysisDescription::ApvLatencyAnalysisDescription(
    uint16_t latency, uint16_t crate, uint16_t slot, uint16_t ring,
    uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr, std::string partition,
    uint32_t runNumber, bool valid, std::string comments, uint16_t fedId,
    uint16_t feUnit, uint16_t feChan, uint16_t fedApv) {
  THROW_RUNTIME;
};

// class CalibrationAnalysisDescription : public
// CommissioningAnalysisDescription {
class CalibrationAnalysisDescription {
 public:
  CalibrationAnalysisDescription(
      float amplitude, float tail, float riseTime, float timeConstant,
      float smearing, float chi2, bool deconvMode, uint16_t crate,
      uint16_t slot, uint16_t ring, uint16_t ccuAdr, uint16_t ccuChan,
      uint16_t i2cAddr, std::string partition, uint32_t runNumber, bool valid,
      std::string comments, uint16_t fedId, uint16_t feUnit, uint16_t feChan,
      uint16_t fedApv, uint16_t calChan, uint16_t isha, uint16_t vfs);
};

CalibrationAnalysisDescription::CalibrationAnalysisDescription(
    float amplitude, float tail, float riseTime, float timeConstant,
    float smearing, float chi2, bool deconvMode, uint16_t crate, uint16_t slot,
    uint16_t ring, uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr,
    std::string partition, uint32_t runNumber, bool valid, std::string comments,
    uint16_t fedId, uint16_t feUnit, uint16_t feChan, uint16_t fedApv,
    uint16_t calChan, uint16_t isha, uint16_t vfs) {
  THROW_RUNTIME;
};

// class FastFedCablingAnalysisDescription : public
// CommissioningAnalysisDescription {
class FastFedCablingAnalysisDescription {
 public:
  FastFedCablingAnalysisDescription(
      float highLevel, float highRms, float lowLevel, float lowRms, float maxll,
      float minll, uint32_t dcuId, uint16_t lldCh, bool isDirty,
      float threshold, float dirtyThreshold, uint16_t crate, uint16_t slot,
      uint16_t ring, uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr,
      std::string partition, uint32_t runNumber, bool valid,
      std::string comments, uint16_t fedId, uint16_t feUnit, uint16_t feChan,
      uint16_t fedApv);
};

FastFedCablingAnalysisDescription::FastFedCablingAnalysisDescription(
    float highLevel, float highRms, float lowLevel, float lowRms, float maxll,
    float minll, uint32_t dcuId, uint16_t lldCh, bool isDirty, float threshold,
    float dirtyThreshold, uint16_t crate, uint16_t slot, uint16_t ring,
    uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr, std::string partition,
    uint32_t runNumber, bool valid, std::string comments, uint16_t fedId,
    uint16_t feUnit, uint16_t feChan, uint16_t fedApv) {
  THROW_RUNTIME;
};

// class FineDelayAnalysisDescription : public CommissioningAnalysisDescription
// {
class FineDelayAnalysisDescription {
  FineDelayAnalysisDescription(float maximum, float error, uint16_t crate,
                               uint16_t slot, uint16_t ring, uint16_t ccuAdr,
                               uint16_t ccuChan, uint16_t i2cAddr,
                               std::string partition, uint32_t runNumber,
                               bool valid, std::string comments, uint16_t fedId,
                               uint16_t feUnit, uint16_t feChan,
                               uint16_t fedApv);
};

FineDelayAnalysisDescription::FineDelayAnalysisDescription(
    float maximum, float error, uint16_t crate, uint16_t slot, uint16_t ring,
    uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr, std::string partition,
    uint32_t runNumber, bool valid, std::string comments, uint16_t fedId,
    uint16_t feUnit, uint16_t feChan, uint16_t fedApv) {
  THROW_RUNTIME;
};

// class laserdriverDescription: public deviceDescription {
class laserdriverDescription {
 public:
  tscType8 getGain();
  void setGain(tscType8 channel, tscType8 gain);
  void getBias(tscType8 *bias);
  void setBias(tscType8 channel, tscType8 bias);
  tscType8 getBias(tscType8 channel);
  tscType8 getGain(tscType8 channel);
  virtual ~laserdriverDescription();
};

tscType8 laserdriverDescription::getGain() { THROW_RUNTIME; };
void laserdriverDescription::setGain(tscType8 channel, tscType8 gain) {
  THROW_RUNTIME;
};
void laserdriverDescription::getBias(tscType8 *bias) { THROW_RUNTIME; };
void laserdriverDescription::setBias(tscType8 channel, tscType8 bias) {
  THROW_RUNTIME;
};
tscType8 laserdriverDescription::getBias(tscType8 channel) { THROW_RUNTIME; };
tscType8 laserdriverDescription::getGain(tscType8 channel) { THROW_RUNTIME; };
laserdriverDescription::~laserdriverDescription() { THROW_RUNTIME; };

// class OptoScanAnalysisDescription : public CommissioningAnalysisDescription {
class OptoScanAnalysisDescription {
 public:
  OptoScanAnalysisDescription(
      float baseLineSlop0, float baseLineSlop1, float baseLineSlop2,
      float baseLineSlop3, uint16_t gain, uint16_t bias0, uint16_t bias1,
      uint16_t bias2, uint16_t bias3, float measGain0, float measGain1,
      float measGain2, float measGain3, float zeroLight0, float zeroLight1,
      float zeroLight2, float zeroLight3, float linkNoise0, float linkNoise1,
      float linkNoise2, float linkNoise3, float liftOff0, float liftOff1,
      float liftOff2, float liftOff3, float threshold0, float threshold1,
      float threshold2, float threshold3, float tickHeight0, float tickHeight1,
      float tickHeight2, float tickHeight3, uint16_t crate, uint16_t slot,
      uint16_t ring, uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr,
      std::string partition, uint32_t runNumber, bool valid,
      std::string comments, uint16_t fedId, uint16_t feUnit, uint16_t feChan,
      uint16_t fedApv);
};

OptoScanAnalysisDescription::OptoScanAnalysisDescription(
    float baseLineSlop0, float baseLineSlop1, float baseLineSlop2,
    float baseLineSlop3, uint16_t gain, uint16_t bias0, uint16_t bias1,
    uint16_t bias2, uint16_t bias3, float measGain0, float measGain1,
    float measGain2, float measGain3, float zeroLight0, float zeroLight1,
    float zeroLight2, float zeroLight3, float linkNoise0, float linkNoise1,
    float linkNoise2, float linkNoise3, float liftOff0, float liftOff1,
    float liftOff2, float liftOff3, float threshold0, float threshold1,
    float threshold2, float threshold3, float tickHeight0, float tickHeight1,
    float tickHeight2, float tickHeight3, uint16_t crate, uint16_t slot,
    uint16_t ring, uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr,
    std::string partition, uint32_t runNumber, bool valid, std::string comments,
    uint16_t fedId, uint16_t feUnit, uint16_t feChan, uint16_t fedApv) {
  THROW_RUNTIME;
};

// class PedestalsAnalysisDescription : public CommissioningAnalysisDescription
// {
class PedestalsAnalysisDescription {
 public:
  PedestalsAnalysisDescription(
      VInt16 dead, VInt16 noisy, float pedsMean, float pedsSpread,
      float noiseMean, float noiseSpread, float rawMean, float rawSpread,
      float pedsMax, float pedsMin, float noiseMax, float noiseMin,
      float rawMax, float rawMin, uint16_t crate, uint16_t slot, uint16_t ring,
      uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr,
      std::string partition, uint32_t runNumber, bool valid,
      std::string comments, uint16_t fedId, uint16_t feUnit, uint16_t feChan,
      uint16_t fedApv);
};

PedestalsAnalysisDescription::PedestalsAnalysisDescription(
    VInt16 dead, VInt16 noisy, float pedsMean, float pedsSpread,
    float noiseMean, float noiseSpread, float rawMean, float rawSpread,
    float pedsMax, float pedsMin, float noiseMax, float noiseMin, float rawMax,
    float rawMin, uint16_t crate, uint16_t slot, uint16_t ring, uint16_t ccuAdr,
    uint16_t ccuChan, uint16_t i2cAddr, std::string partition,
    uint32_t runNumber, bool valid, std::string comments, uint16_t fedId,
    uint16_t feUnit, uint16_t feChan, uint16_t fedApv) {
  THROW_RUNTIME;
};

// class pllDescription: public deviceDescription {
class pllDescription {
 public:
  tscType8 getDelayFine();
  tscType8 getDelayCoarse();
  void setDelayFine(tscType8 clockPhase);
  void setDelayCoarse(tscType8 triggerDelay);
  virtual ~pllDescription();
};

tscType8 pllDescription::getDelayFine() { THROW_RUNTIME; };
tscType8 pllDescription::getDelayCoarse() { THROW_RUNTIME; };
void pllDescription::setDelayFine(tscType8 clockPhase) { THROW_RUNTIME; };
void pllDescription::setDelayCoarse(tscType8 triggerDelay) { THROW_RUNTIME; };
pllDescription::~pllDescription() { THROW_RUNTIME; };

// class VpspScanAnalysisDescription : public CommissioningAnalysisDescription {
class VpspScanAnalysisDescription {
 public:
  VpspScanAnalysisDescription(
      uint16_t vpsp, uint16_t adcLevel, uint16_t fraction, uint16_t topEdge,
      uint16_t bottomEdge, uint16_t topLevel, uint16_t bottomLevel,
      uint16_t crate, uint16_t slot, uint16_t ring, uint16_t ccuAdr,
      uint16_t ccuChan, uint16_t i2cAddr, std::string partition,
      uint32_t runNumber, bool valid, std::string comments, uint16_t fedId,
      uint16_t feUnit, uint16_t feChan, uint16_t fedApv);
};

VpspScanAnalysisDescription::VpspScanAnalysisDescription(
    uint16_t vpsp, uint16_t adcLevel, uint16_t fraction, uint16_t topEdge,
    uint16_t bottomEdge, uint16_t topLevel, uint16_t bottomLevel,
    uint16_t crate, uint16_t slot, uint16_t ring, uint16_t ccuAdr,
    uint16_t ccuChan, uint16_t i2cAddr, std::string partition,
    uint32_t runNumber, bool valid, std::string comments, uint16_t fedId,
    uint16_t feUnit, uint16_t feChan, uint16_t fedApv) {
  THROW_RUNTIME;
};



typedef std::vector<float> VFloat;

class PedsFullNoiseAnalysisDescription {
public:

PedsFullNoiseAnalysisDescription ();


PedsFullNoiseAnalysisDescription (
 VInt16 deadStrips, VInt16 badStrips, VInt16 shiftedStrips,
 VInt16 lowNoiseStrips, VInt16 largeNoiseStrips, VInt16 largeNoiseSignificance,
 VInt16 badFitStatus, VInt16 badADProbab, VInt16 badKSProbab,
 VInt16 badJBProbab, VInt16 badChi2Probab, VInt16 badTailStrips,
 VInt16 badDoublePeakStrips, VFloat adProbab, VFloat ksProbab,
 VFloat jbProbab, VFloat chi2Probab, VFloat noiseRMS,
 VFloat noiseSigmaGaus, VFloat noiseSignificance, VFloat residualSkewness, VFloat residualKurtosis,
 VFloat residualIntegralNsigma, VFloat residualIntegral, uint16_t crate, uint16_t slot,
 uint16_t ring, uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr,
 std::string partition, uint32_t runNumber, bool valid,
 std::string comments, uint16_t fedId, uint16_t feUnit, uint16_t feChan, uint16_t fedApv);

PedsFullNoiseAnalysisDescription
(uint16_t crate,uint16_t slot,uint16_t ring,
 uint16_t ccuAdr,uint16_t ccuChan,uint16_t i2cAddr,std::string partition,uint32_t runNumber,bool valid,std::string comments,uint16_t fedId,uint16_t feUnit,uint16_t feChan,uint16_t fedApv);

  ~PedsFullNoiseAnalysisDescription();

};

PedsFullNoiseAnalysisDescription::PedsFullNoiseAnalysisDescription () { THROW_RUNTIME; };

PedsFullNoiseAnalysisDescription::PedsFullNoiseAnalysisDescription (
 VInt16 deadStrips, VInt16 badStrips, VInt16 shiftedStrips,
 VInt16 lowNoiseStrips, VInt16 largeNoiseStrips, VInt16 largeNoiseSignificance,
 VInt16 badFitStatus, VInt16 badADProbab, VInt16 badKSProbab,
 VInt16 badJBProbab, VInt16 badChi2Probab, VInt16 badTailStrips,
 VInt16 badDoublePeakStrips, VFloat adProbab, VFloat ksProbab,
 VFloat jbProbab, VFloat chi2Probab, VFloat noiseRMS,
 VFloat noiseSigmaGaus, VFloat noiseSignificance, VFloat residualSkewness, VFloat residualKurtosis,
 VFloat residualIntegralNsigma, VFloat residualIntegral, uint16_t crate, uint16_t slot,
 uint16_t ring, uint16_t ccuAdr, uint16_t ccuChan, uint16_t i2cAddr,
 std::string partition, uint32_t runNumber, bool valid,
 std::string comments, uint16_t fedId, uint16_t feUnit, uint16_t feChan, uint16_t fedApv ) {
  THROW_RUNTIME; 
};

PedsFullNoiseAnalysisDescription::PedsFullNoiseAnalysisDescription
(uint16_t crate,uint16_t slot,uint16_t ring,
 uint16_t ccuAdr,uint16_t ccuChan,uint16_t i2cAddr,std::string partition,uint32_t runNumber,bool valid,std::string comments,uint16_t fedId,uint16_t feUnit,uint16_t feChan,uint16_t fedApv) { 
  THROW_RUNTIME;  
};

PedsFullNoiseAnalysisDescription::~PedsFullNoiseAnalysisDescription () { THROW_RUNTIME; };

