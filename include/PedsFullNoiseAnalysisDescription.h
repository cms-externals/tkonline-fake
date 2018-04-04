#ifndef PEDSFULLNOISEANALYSISDESCRIPTION_H
#define PEDSFULLNOISEANALYSISDESCRIPTION_H
 
#include "PedsFullNoiseAnalysisDescription.h"
#include "ParameterDescription.h"
#include "CommissioningAnalysisDescription.h"
#include <algorithm>
#include <iomanip>
 
/**
   @class PedsFullNoiseAnalysisDescription.h
   @author Raffaele Angelo Gerosa
   @brief  Peds full noise run for Bad strips
*/
     class PedsFullNoiseAnalysisDescription : public CommissioningAnalysisDescription {
 
 
 private:
 
  // properties: 
  /** Dead strips values are strips numbers. */
  VInt16 _deadStrips;
  /** bad strips values are strips numbers. */
  VInt16 _badStrips;
  /** bad strips shifted */
  VInt16 _shiftedStrips;
  /** bad strips low noisy */
  VInt16 _lowNoiseStrips;
  /** bad strips large noisy */
  VInt16 _largeNoiseStrips;
  /** bad strips large noise signif */
  VInt16 _largeNoiseSignifStrips;
  /** bad strips fit status */
  VInt16 _badFitStrips;
  /** bad AD probab */
  VInt16 _badADProbabStrips;
  /** bad KS probab */
  VInt16 _badKSProbabStrips;
  /** bad JB probab */
  VInt16 _badJBProbabStrips;
  /** bad Chi2 probab */
  VInt16 _badChi2ProbabStrips;
  /** bad tail probab */
  VInt16 _badTailStrips;
  /** bad cause of double peak */
  VInt16 _badDoublePeakStrips;
 
  /** List of bad strips */
  VInt16 _deadStripBit;
  VInt16 _badStripBit;
  VInt16 _reasonBadStripBit;
  
  /// --> test statistic values per strips                                                                                                                          
  VFloat _adProbab;
  VFloat _ksProbab;
  VFloat _jbProbab;
  VFloat _chi2Probab;
  VFloat _noiseRMS;
  VFloat _noiseSigmaGaus;
  VFloat _noiseSignificance;
  VFloat _residualSkewness;
  VFloat _residualKurtosis;
  VFloat _residualIntegralNsigma;
  VFloat _residualIntegral;
  VFloat _fractionOverN;
 
 public:
 
  // parameters list:
 
  /** @brief Enum of parameter names */  
  enum{DEAD,BAD,SHIFT,LOWNOISE,LARGENOISE,LARGESIGNIF,BADFIT,BADADPROB,BADKSPROB,BADJBPROB,BADCHI2PROB,BADTAIL,BADDOUBLEPEAK,DEADBIT,BADBIT,REASONBIT,
       ADPROBAB,KSPROBAB,JBPROBAB,CHI2PROBAB,NOISERMS,NOISESIGMAGAUS,NOISESIGNIF,RESSKEWNESS,RESKURTOSIS,RESINTEGRAL,RESINTEGRALNSIGMA,FRACTIONOVERN         
  };
  /** @brief List of parameter names */
  static const char *PEDSFULLNOISEANALYSISDESCRIPTION[FRACTIONOVERN+1];
 
 
  // con(de)structors:
 
  /** @brief default constructor */
  PedsFullNoiseAnalysisDescription();
 
  PedsFullNoiseAnalysisDescription(VInt16 deadStrips,
                                   VInt16 badStrips,
                                   VInt16 shiftedStrips,
                                   VInt16 lowNoiseStrips,
                                   VInt16 largeNoiseStrips,
                                   VInt16 largeNoiseSignificance,
                                   VInt16 badFitStatus,
                                   VInt16 badADProbab,
                                   VInt16 badKSProbab,
                                   VInt16 badJBProbab,
                                   VInt16 badChi2Probab,
                                   VInt16 badTailStrips,
                                   VInt16 badDoublePeakStrips,
                                   VFloat adProbab,
                                   VFloat ksProbab,
                                   VFloat jbProbab,
                                   VFloat chi2Probab,
                                   VFloat noiseRMS,
                                   VFloat noiseSigmaGaus,
                                   VFloat noiseSignificance,
                                   VFloat residualSkewness,
                                   VFloat residualKurtosis,
                                   VFloat residualIntegralNsigma,
                                   VFloat residualIntegral,
                                   uint16_t crate,
                                   uint16_t slot,
                                   uint16_t ring,
                                   uint16_t ccuAdr,
                                   uint16_t ccuChan,
                                   uint16_t i2cAddr,
                                   std::string partition,
                                   uint32_t runNumber,
                                   bool valid,
                                   std::string comments,
                                   uint16_t fedId,
                                   uint16_t feUnit,
                                   uint16_t feChan,
                                   uint16_t fedApv);
 
 
 
  PedsFullNoiseAnalysisDescription(uint16_t crate,
                                   uint16_t slot,
                                   uint16_t ring,
                                   uint16_t ccuAdr,
                                   uint16_t ccuChan,
                                   uint16_t i2cAddr,
                                   std::string partition,
                                   uint32_t runNumber,
                                   bool valid,
                                   std::string comments,
                                   uint16_t fedId,
                                   uint16_t feUnit,
                                   uint16_t feChan,
                                   uint16_t fedApv);
 
 
  PedsFullNoiseAnalysisDescription(parameterDescriptionNameType parameterNames);
       
 
 
  // methods:
 
  /** @brief human readable object content */
  virtual std::string toString();
  /** @brief clone method */
  virtual CommissioningAnalysisDescription *clone();
  /** @brief display object content in standard output */
  virtual void display();
  /** @brief get the commissioning analysis type (see commissioningType) */
  virtual CommissioningAnalysisDescription::commissioningType getType() const { return CommissioningAnalysisDescription::T_ANALYSIS_PEDSFULLNOISE; }
  /** @brief get the list of parameter name (XML clob interface) */
  static parameterDescriptionNameType *getParameterNames();
 
 
  // getters / setters:
       
  /** @brief get human readable analysis type */
  std::string getAnalysisType() const;
 
  /** @brief set: Dead strips values are strips numbers. */
  void setDeadStrips( const VInt16 value );
  /** @brief set: Dead strips values are strips numbers concatened and separated by '|'. */
  void setDeadStrips( const std::string &value );
  /** @brief get: Dead strips values are strips numbers concatened and separated by '|'. */
  std::string getDeadStrips() const;
  /** @brief get: Dead strips values are strips numbers. */
  void getDeadStrips( VInt16 &value );
 
  /** @brief set: Bad strips values are strips numbers. */
  void setBadStrips( const VInt16 value );
  /** @brief set: Bad strips values are strips numbers concatened and separated by '|'. */
  void setBadStrips( const std::string &value );
  /** @brief get: Bad strips values are strips numbers concatened and separated by '|'. */
  std::string getBadStrips() const;
  /** @brief get: Bad strips values are strips numbers. */
  void getBadStrips( VInt16 &value );
 
  /** @brief set: Shifted strips values are strips numbers. */
  void setShiftedStrips( const VInt16 value );
  /** @brief set: Shifted strips values are strips numbers concatened and separated by '|'. */
  void setShiftedStrips( const std::string &value );
  /** @brief get: Shifted strips values are strips numbers concatened and separated by '|'. */
  std::string getShiftedStrips() const;
  /** @brief get: Shifted strips values are strips numbers. */
  void getShiftedStrips( VInt16 &value );
 
  /** @brief set: LowNoise strips values are strips numbers. */
  void setLowNoiseStrips( const VInt16 value );
  /** @brief set: LowNoise strips values are strips numbers concatened and separated by '|'. */
  void setLowNoiseStrips( const std::string &value );
  /** @brief get: LowNoise strips values are strips numbers concatened and separated by '|'. */
  std::string getLowNoiseStrips() const;
  /** @brief get: LowNoise strips values are strips numbers. */
  void getLowNoiseStrips( VInt16 &value );
 
  /** @brief set: LargeNoise strips values are strips numbers. */
  void setLargeNoiseStrips( const VInt16 value );
  /** @brief set: LargeNoise strips values are strips numbers concatened and separated by '|'. */
  void setLargeNoiseStrips( const std::string &value );
  /** @brief get: LargeNoise strips values are strips numbers concatened and separated by '|'. */
  std::string getLargeNoiseStrips() const;
  /** @brief get: LargeNoise strips values are strips numbers. */
  void getLargeNoiseStrips( VInt16 &value );
 
  /** @brief set: LargeSignif strips values are strips numbers. */
  void setLargeNoiseSignifStrips( const VInt16 value );
  /** @brief set: LargeSignif strips values are strips numbers concatened and separated by '|'. */
  void setLargeNoiseSignifStrips( const std::string &value );
  /** @brief get: LargeSignif strips values are strips numbers concatened and separated by '|'. */
  std::string getLargeNoiseSignifStrips() const;
  /** @brief get: LargeSignif strips values are strips numbers. */
  void getLargeNoiseSignifStrips( VInt16 &value );
 
  /** @brief set: BadFit strips values are strips numbers. */
  void setBadFitStrips( const VInt16 value );
  /** @brief set: BadFit strips values are strips numbers concatened and separated by '|'. */
  void setBadFitStrips( const std::string &value );
  /** @brief get: BadFit strips values are strips numbers concatened and separated by '|'. */
  std::string getBadFitStrips() const;
  /** @brief get: BadFit strips values are strips numbers. */
  void getBadFitStrips( VInt16 &value );
 
  /** @brief set: BadADProbab strips values are strips numbers. */
  void setBadADProbabStrips( const VInt16 value );
  /** @brief set: BadADProbab strips values are strips numbers concatened and separated by '|'. */
  void setBadADProbabStrips( const std::string &value );
  /** @brief get: BadADProbab strips values are strips numbers concatened and separated by '|'. */
  std::string getBadADProbabStrips() const;
  /** @brief get: BadADProbab strips values are strips numbers. */
  void getBadADProbabStrips( VInt16 &value );
 
  /** @brief set: BadKSProbab strips values are strips numbers. */
  void setBadKSProbabStrips( const VInt16 value );
  /** @brief set: BadKSProbab strips values are strips numbers concatened and separated by '|'. */
  void setBadKSProbabStrips( const std::string &value );
  /** @brief get: BadKSProbab strips values are strips numbers concatened and separated by '|'. */
  std::string getBadKSProbabStrips() const;
  /** @brief get: BadKSProbab strips values are strips numbers. */
  void getBadKSProbabStrips( VInt16 &value );
 
  /** @brief set: BadJBProbab strips values are strips numbers. */
  void setBadJBProbabStrips( const VInt16 value );
  /** @brief set: BadJBProbab strips values are strips numbers concatened and separated by '|'. */
  void setBadJBProbabStrips( const std::string &value );
  /** @brief get: BadJBProbab strips values are strips numbers concatened and separated by '|'. */
  std::string getBadJBProbabStrips() const;
  /** @brief get: BadJBProbab strips values are strips numbers. */
  void getBadJBProbabStrips( VInt16 &value );
 
  /** @brief set: BadChi2Probab strips values are strips numbers. */
  void setBadChi2ProbabStrips( const VInt16 value );
  /** @brief set: BadChi2Probab strips values are strips numbers concatened and separated by '|'. */
  void setBadChi2ProbabStrips( const std::string &value );
  /** @brief get: BadChi2Probab strips values are strips numbers concatened and separated by '|'. */
  std::string getBadChi2ProbabStrips() const;
  /** @brief get: BadChi2Probab strips values are strips numbers. */
  void getBadChi2ProbabStrips( VInt16 &value );
 
  /** @brief set: BadTail strips values are strips numbers. */
  void setBadTailStrips( const VInt16 value );
  /** @brief set: BadTail strips values are strips numbers concatened and separated by '|'. */
  void setBadTailStrips( const std::string &value );
  /** @brief get: BadTail strips values are strips numbers concatened and separated by '|'. */
  std::string getBadTailStrips() const;
  /** @brief get: BadTail strips values are strips numbers. */
  void getBadTailStrips( VInt16 &value );
 
  /** @brief set: BadDoublePeak strips values are strips numbers. */
  void setBadDoublePeakStrips( const VInt16 value );
  /** @brief set: BadDoublePeak strips values are strips numbers concatened and separated by '|'. */
  void setBadDoublePeakStrips( const std::string &value );
  /** @brief get: BadDoublePeak strips values are strips numbers concatened and separated by '|'. */
  std::string getBadDoublePeakStrips() const;
  /** @brief get: BadDoublePeak strips values are strips numbers. */
  void getBadDoublePeakStrips( VInt16 &value );
  
  /** @brief set: ADProbab strips values are strips numbers. */
  void setADProbab( const VFloat value );
  /** @brief set: ADProbab strips values are strips numbers concatened and separated by '|'. */
  void setADProbab( const std::string &value );
  /** @brief get: ADProbab strips values are strips numbers concatened and separated by '|'. */
  std::string getADProbab() const;
  /** @brief get: ADProbab strips values are strips numbers. */
  void getADProbab( VFloat &value );
 
  /** @brief set: KSProbab strips values are strips numbers. */
  void setKSProbab( const VFloat value );
  /** @brief set: KSProbab strips values are strips numbers concatened and separated by '|'. */
  void setKSProbab( const std::string &value );
  /** @brief get: KSProbab strips values are strips numbers concatened and separated by '|'. */
  std::string getKSProbab() const;
  /** @brief get: KSProbab strips values are strips numbers. */
  void getKSProbab( VFloat &value );
 
  /** @brief set: JBProbab strips values are strips numbers. */
  void setJBProbab( const VFloat value );
  /** @brief set: JBProbab strips values are strips numbers concatened and separated by '|'. */
  void setJBProbab( const std::string &value );
  /** @brief get: JBProbab strips values are strips numbers concatened and separated by '|'. */
  std::string getJBProbab() const;
  /** @brief get: JBProbab strips values are strips numbers. */
  void getJBProbab( VFloat &value );
 
  /** @brief set: Chi2Probab strips values are strips numbers. */
  void setChi2Probab( const VFloat value );
  /** @brief set: Chi2Probab strips values are strips numbers concatened and separated by '|'. */
  void setChi2Probab( const std::string &value );
  /** @brief get: Chi2Probab strips values are strips numbers concatened and separated by '|'. */
  std::string getChi2Probab() const;
  /** @brief get: Chi2Probab strips values are strips numbers. */
  void getChi2Probab( VFloat &value );
 
  /** @brief set: NoiseRMS strips values are strips numbers. */
  void setNoiseRMS( const VFloat value );
  /** @brief set: NoiseRMS strips values are strips numbers concatened and separated by '|'. */
  void setNoiseRMS( const std::string &value );
  /** @brief get: NoiseRMS strips values are strips numbers concatened and separated by '|'. */
  std::string getNoiseRMS() const;
  /** @brief get: NoiseRMS strips values are strips numbers. */
  void getNoiseRMS( VFloat &value );
 
  /** @brief set: NoiseSigmaGaus strips values are strips numbers. */
  void setNoiseSigmaGaus( const VFloat value );
  /** @brief set: NoiseSigmaGaus strips values are strips numbers concatened and separated by '|'. */
  void setNoiseSigmaGaus( const std::string &value );
  /** @brief get: NoiseSigmaGaus strips values are strips numbers concatened and separated by '|'. */
  std::string getNoiseSigmaGaus() const;
  /** @brief get: NoiseSigmaGaus strips values are strips numbers. */
  void getNoiseSigmaGaus( VFloat &value );
 
  /** @brief set: NoiseSignificance strips values are strips numbers. */
  void setNoiseSignificance( const VFloat value );
  /** @brief set: NoiseSignificance strips values are strips numbers concatened and separated by '|'. */
  void setNoiseSignificance( const std::string &value );
  /** @brief get: NoiseSignificance strips values are strips numbers concatened and separated by '|'. */
  std::string getNoiseSignificance() const;
  /** @brief get: NoiseSignificance strips values are strips numbers. */
  void getNoiseSignificance( VFloat &value );
 
  /** @brief set: ResidualSkewness strips values are strips numbers. */
  void setResidualSkewness( const VFloat value );
  /** @brief set: ResidualSkewness strips values are strips numbers concatened and separated by '|'. */
  void setResidualSkewness( const std::string &value );
  /** @brief get: ResidualSkewness strips values are strips numbers concatened and separated by '|'. */
  std::string getResidualSkewness() const;
  /** @brief get: ResidualSkewness strips values are strips numbers. */
  void getResidualSkewness( VFloat &value );
 
  /** @brief set: ResidualKurtosis strips values are strips numbers. */
  void setResidualKurtosis( const VFloat value );
  /** @brief set: ResidualKurtosis strips values are strips numbers concatened and separated by '|'. */
  void setResidualKurtosis( const std::string &value );
  /** @brief get: ResidualKurtosis strips values are strips numbers concatened and separated by '|'. */
  std::string getResidualKurtosis() const;
  /** @brief get: ResidualKurtosis strips values are strips numbers. */
  void getResidualKurtosis( VFloat &value );
 
  /** @brief set: ResidualIntegral strips values are strips numbers. */
  void setResidualIntegral( const VFloat value );
  /** @brief set: ResidualIntegral strips values are strips numbers concatened and separated by '|'. */
  void setResidualIntegral( const std::string &value );
  /** @brief get: ResidualIntegral strips values are strips numbers concatened and separated by '|'. */
  std::string getResidualIntegral() const;
  /** @brief get: ResidualIntegral strips values are strips numbers. */
  void getResidualIntegral( VFloat &value );
 
  /** @brief set: ResidualIntegralNsigma strips values are strips numbers. */
  void setResidualIntegralNsigma( const VFloat value );
  /** @brief set: ResidualIntegralNsigma strips values are strips numbers concatened and separated by '|'. */
  void setResidualIntegralNsigma( const std::string &value );
  /** @brief get: ResidualIntegralNsigma strips values are strips numbers concatened and separated by '|'. */
  std::string getResidualIntegralNsigma() const;
  /** @brief get: ResidualIntegralNsigma strips values are strips numbers. */
  void getResidualIntegralNsigma( VFloat &value );
 
  // internal methods  
  void setDeadStripBit();
  void setBadStripBit();
  void setReasonBadStripBit();
  void setFractionOverN();
 
  void setDeadStripBit( const std::string &value );
  void setBadStripBit( const std::string &value );
  void setReasonBadStripBit( const std::string &value );
  void setFractionOverN( const std::string &value );
 
  void setDeadStripBit( const VInt16 value );
  void setBadStripBit( const VInt16 value );
  void setReasonBadStripBit( const VInt16 value );
  void setFractionOverN( const VFloat value );
 
  std::string getDeadStripBit() const;
  std::string getBadStripBit() const;
  std::string getReasonBadStripBit() const;
  std::string getFractionOverN() const;
 
  void getDeadStripBit(VInt16 & value);
  void getBadStripBit(VInt16 & value);
  void getReasonBadStripBit(VInt16 & value);
  void getFractionOverN(VFloat & value);
 
};
 
#endif
