#pragma once


template<class T> class ComplementaryFilter {

  T m_WeightNew;

  // Current filtered value. 
  T m_Current;

public:
  ComplementaryFilter(T WeightNew, T Initial)
    : m_WeightNew(WeightNew), m_Current(Initial)
  { }

  void Filter(T New)
  {
    m_Current = (m_WeightNew * New + (100 - m_WeightNew) * m_Current) / 100;
  }

  void SetWeight(T NewWeight)
  {
    m_WeightNew = NewWeight;
  }

  T GetWeight() const { return m_WeightNew; }

  T Current() const { return m_Current; }
};

// Specialization for floating point math. 
template<> class ComplementaryFilter<float>
{
  float m_fWeightNew;
  float m_fCurrent;

public:
  ComplementaryFilter(float fWeightNew, float fInitial)
    : m_fWeightNew(fWeightNew/100.0), m_fCurrent(fInitial)
  { }


  void Filter(float fNew)
  {
    m_fCurrent = m_fWeightNew * fNew + (1.0 - m_fWeightNew) * m_fCurrent;
  }

  void SetWeight(float NewWeight)
  {
    m_fWeightNew = NewWeight/100.0;
  }

  float GetWeight() const { return m_fWeightNew*100.0; }

  float Current() const { return m_fCurrent; }
};
