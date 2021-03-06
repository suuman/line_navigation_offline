/*
This file is part of the BIAS library (Basic ImageAlgorithmS).

Copyright (C) 2003-2009    (see file CONTACT for details)
  Multimediale Systeme der Informationsverarbeitung
  Institut fuer Informatik
  Christian-Albrechts-Universitaet Kiel


BIAS is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 2.1 of the License, or
(at your option) any later version.

BIAS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with BIAS; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#ifndef __FilterNTo2N_hh__
#define __FilterNTo2N_hh__

#include "FilterBase.hh"


namespace BIAS 
{
  /** @class FilterNTo2N
      @ingroup g_filter
      @brief base class for simple n->2n filter implementations
      
      This is the base class for n->2n filters, i.e. for every input channel 
      two output channels are generated by filters derived from this class.
      
      @author skoglund, koeser, woelk 11/2004 */ 

  template <class InputStorageType, class OutputStorageType>
  class BIASFilter_EXPORT FilterNTo2N : 
    public FilterBase<InputStorageType, OutputStorageType>
  {
  public:
    /** algorithm used to calculate the absolute length of the 2D vector
        - VLT_L1:  | fabs(gx) + fabs(gy) |
        - VLT_L2:  sqrt(gx*gx+gy*gy)
        - VLT_max: max(fabs(gx), fabs(gy)) */
    enum EVecLengthType { VLT_L1, VLT_L2, VLT_max };
  public:
    FilterNTo2N();
    FilterNTo2N(const FilterBase<InputStorageType, OutputStorageType>& other);
    virtual ~FilterNTo2N();
    
    /** dst.GetChannelCount()==2*src.GetCHannelCount() */
    virtual int Filter(const Image<InputStorageType>& src, 
                       Image<OutputStorageType>& dst);
    virtual int FilterInt(const Image<InputStorageType>& src, 
                          Image<OutputStorageType>& dst);
    virtual int FilterFloat(const Image<InputStorageType>& src, 
                            Image<OutputStorageType>& dst);

    /** dstX.GetChannelCount()==src.GetCHannelCount() */
    virtual int Filter(const Image<InputStorageType>& src, 
                       Image<OutputStorageType>& dst1, 
                       Image<OutputStorageType>& dst2);
    virtual int FilterInt(const Image<InputStorageType>& src, 
                          Image<OutputStorageType>& dst1, 
                          Image<OutputStorageType>& dst2);
    virtual int FilterFloat(const Image<InputStorageType>& src, 
                            Image<OutputStorageType>& dst1, 
                            Image<OutputStorageType>& dst2);
    virtual FilterNTo2N<InputStorageType, OutputStorageType>* Clone() const = 0;

    /** decides which vector length to use from *_VecLenthType
        @author woelk 12/2004 */
    int VecLen(const Image<OutputStorageType>& gx, 
               const Image<OutputStorageType>& gy,
               Image<OutputStorageType>& length);

    /** length = | fabs(gx) + fabs(gy) |  
        destination absg must be initialized
        @author woelk 09/2004 */
    int VecLenL1(const Image<OutputStorageType>& gx, 
                 const Image<OutputStorageType>& gy,
                 Image<OutputStorageType>& length);

    /** length = sqrt(gx*gx+gy*gy)  
        destination absg must be initialized
        @author woelk 09/2004 */
    int VecLenL2(const Image<OutputStorageType>& gx, 
                 const Image<OutputStorageType>& gy,
                  Image<OutputStorageType>& length);

    /** length = max(fabs(gx),fabs(gy))  
        destination absg must be initialized
        @author woelk 09/2004 */
    int VecLenMax(const Image<OutputStorageType>& gx, 
                  const Image<OutputStorageType>& gy,
                  Image<OutputStorageType>& length);

  protected:
    /// of type Gradient<InputStorageType, OutputStorageType>::EVecLengthType
    int _VecLengthType; 
  }; // class
} // namespace

#endif // __FilterNTo2N_hh__
