

#ifndef __vrutils_math_h__
#define __vrutils_math_h__


#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

namespace vrutils {

  /// Lineary interpolate from a to b using s = {0,..1}
  template<typename T>
  inline T const mix(T const& a, T const& b, float s)
  {
    if (s > 1) 
      return b;
    if (s <=0)
      return a;

    return (a*(1-s)  + b*s);
  }

  /*! Cubical interpolation between a and b using s = {0,..1}
  Cubic function expressed according to Horners rule
  */
  template<typename T>
  inline T const smoothstep(T const& a, T const& b, T s)
  {
    if (s >= 1) 
      return b;
    if (s <0)
      return a;
    s = (s-a) / (b-a);
    return(s*s * (3-2*s));
  }


  /// Return the smallest item of a and b
  template<typename T>
    inline T const& min(T const& a, T const& b)
  {
    return (a < b ? a : b);
  }

  /// Return the smallest item of a and b and c
  template<typename T>
    inline T const& min(T const& a, T const& b, T const& c)
  {
    return min(a, min(b,c));
  }

  /// Return the greatest item of a and b
  template<typename T>
    inline T const& max(T const& a, T const& b)
  {
    return (a > b ? a : b);
  }

  /// Return the greatest item of a and b and c
  template<typename T>
    inline T const& max(T const& a, T const& b, T const& c)
  {
    return max(a, max(b,c));
  }


} // namespace vrutils

#endif
