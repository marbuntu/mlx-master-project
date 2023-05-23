/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 CTTC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mattia Sandri
 */

#include <ns3/log.h>
#include <ns3/double.h>
#include <math.h>
#include "antenna-model.h"
#include "circular-aperture-antenna-model.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("CircularApertureAntennaModel");

NS_OBJECT_ENSURE_REGISTERED (CircularApertureAntennaModel);

TypeId
CircularApertureAntennaModel::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::CircularApertureAntennaModel")
    .SetParent<AntennaModel> ()
    .SetGroupName ("Antenna")
    .AddConstructor<CircularApertureAntennaModel> ()
  ;
  return tid;
}

CircularApertureAntennaModel::CircularApertureAntennaModel ()
    : AntennaModel ()
{}

CircularApertureAntennaModel::~CircularApertureAntennaModel ()
{}

void
CircularApertureAntennaModel::SetOrientation (Angles a)
{
  NS_LOG_FUNCTION (this << a);
  m_antennaOrientation = a;
}

void
CircularApertureAntennaModel::SetApertureRadius (double r)
{
  NS_LOG_FUNCTION (this << r);
  m_apertureRadius = r;
}

void
CircularApertureAntennaModel::SetOperatingFreqyency (double f)
{
  NS_LOG_FUNCTION (this << f);
  m_operatingFrequency = f;
}

void
CircularApertureAntennaModel::SetMaxGain(double gain)
{
  NS_LOG_FUNCTION (this << gain);
  m_maxGain = gain;
}

double
CircularApertureAntennaModel::GetGainDb (Angles a)
{
  NS_LOG_FUNCTION (this << a);

  double theta = abs( m_antennaOrientation.GetInclination() - a.GetInclination() );  //The angle from the bore sight antenna main beam
  double gain = 0;

  if(theta == 0)
  {
    gain = m_maxGain;
  }
  else //theta =! 0
  {
    double k = ( 2 * M_PI * m_operatingFrequency ) / 299792458;
    double J_1 = std::cyl_bessel_j( 1 , ( k * m_apertureRadius * sin(theta) ) );
    double denominator = k * m_apertureRadius * sin(theta);
    gain = 4 * pow( abs((J_1/denominator)) , 2 );
    gain = 10 * log10(gain) + m_maxGain;
  }

  return gain;
}


}
