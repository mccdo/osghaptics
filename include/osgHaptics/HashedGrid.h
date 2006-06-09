#ifndef __HashedGrid_h__
#define __HashedGrid_h__

#include <osg/Referenced>
#include <osg/BoundingBox>
#include <osg/Vec3>
#include <vector>
#include <set>
#include <map>
#include <osg/ref_ptr>

namespace osgHaptics {
template <class T>

/// A class to hash data into a spatial hashed grid No explicit grid is stored

class HashedGrid : public osg::Referenced
  {
  public:
    HashedGrid(const osg::Vec3& dimension, unsigned int hash_size=100) : m_dimension(dimension),
      m_hash_size(hash_size)
    {
      setHashSize(m_hash_size);
    }
    
    
    struct HashedDataVector : public std::set<T>, public osg::Referenced {
    
      
    };

    typedef std::map< unsigned long, osg::ref_ptr<HashedDataVector> > HashMap;
    typedef std::vector< const HashedDataVector * > HashVector;

    typedef typename HashMap::iterator iterator;
    typedef typename HashMap::const_iterator const_iterator;

    typedef typename HashMap::const_reference const_reference;

    /// Set the resolution of the hashed grid (cell size == dimension/HashSize)
    void setHashSize(unsigned int l) { m_hash_size = l; }
  
    /// Set the dimension of the hashed grid
    void setDimension(const osg::Vec3& dim){ m_dimension = dim; }
    
    osg::BoundingBox getBound() const {
      osg::BoundingBox bbox;
      osg::Vec3 min,max;
      min = m_center - m_dimension*0.5;
      max = m_center + m_dimension*0.5;
      bbox.expandBy(min);
      bbox.expandBy(max);
      return bbox;
    }

    /// Set the center of the Grid
    void setCenter(const osg::Vec3& center){ m_center = center; }

    /// Insert a datapoint into the grid, it will be hashed into the hashvector
    void insert(const osg::Vec3&, T data);

    /*!
      Get a vector with all data items that are in the cell of the point p + all the 26 neighbours
      of this vector.

      \param p - A point that will be used for intersection
      \param result_vector - A vector with data items that are within the neighbouring cells
      of point p.
      \returns true if any data-items were found to be in proximity
    */
    bool intersect(const osg::Vec3& p, HashVector& result_vector) const
    {
      int x,y,z;
      // Now we know at what cell we are, lets find the neighbour cells to.
      calcCell(p, x,y,z);
      //std::cerr << "Pos: " << x << "; " << y << "; " << z << std::endl;

      // All neighbours
      const int table_size = 27;
      int table[table_size][3] ={
        {0,0,0},
        {-1,-1,0},
        {-1,0,0},
        {-1,1,0},

        {0,1,0},
        {1,1,0},
        {1,0,0},

        {1,-1,0},
        {0,-1,0},
        // z=1
        {0,0,1},
        {-1,-1,1},
        {-1,0,1},
        {-1,1,1},

        {0,1,1},
        {1,1,1},
        {1,0,1},

        {1,-1,1},
        {0,-1,1},
        // z=-1
        {0,0,-1},
        {-1,-1,-1},
        {-1,0,-1},
        {-1,1,-1},

        {0,1,-1},
        {1,1,-1},
        {1,0,-1},

        {1,-1,-1},
        {0,-1,-1}     
      };

      // FOr all neighbours, see if we can find any dataitems
      for(int i=0; i < table_size; i++) {
        unsigned int cell[3] = {table[i][0]+x, table[i][1]+y, table[i][2]+z};
        
        if (cell[0] < 0 ||cell[0] > m_hash_size ||
            cell[1] < 0 ||cell[1] > m_hash_size ||
            cell[2] < 0 ||cell[2] > m_hash_size) {
            
              //std::cerr << "Outside: "  << " " << "Cell: " << cell[0] << ", " << cell[1] << ", " << cell[2] << std::endl;
              continue;
        }

        unsigned long idx = computeHashBucketIndex(cell[0],cell[1],cell[2]);

        //std::cerr << "Intersecting dded: " << idx << ": " << cell[0] << ", " << cell[1] << ", " << cell[2] << std::endl;
        const_iterator it = m_hash_vector.find(idx);
        if (it != m_hash_vector.end()) {
          if (it->second->size())
            result_vector.push_back(it->second.get());
        }
      }
          
      return result_vector.size() > 0;
    }

    unsigned long computeHashBucketIndex(      
      const  int& x, 
      const  int& y, 
      const  int& z) const;
  
    void calcCell(const osg::Vec3& p, int& x, int& y, int& z) const
    {
      // calculate the integer position from the float position
      float dx = m_dimension[0] / m_hash_size; 
      float dy = m_dimension[1] / m_hash_size; 
      float dz = m_dimension[2] / m_hash_size; 

      // Now calculate which voxel we are using here.
      x = (p[0]-m_center[0]+m_dimension[0]*0.5)/dx;
      y = (p[1]-m_center[1]+m_dimension[1]*0.5)/dy;
      z = (p[2]-m_center[2]+m_dimension[2]*0.5)/dz;

    }

    iterator begin() { return m_hash_vector.begin(); }
    iterator end() { return m_hash_vector.end(); }

    const_iterator begin() const { return m_hash_vector.begin(); }
    const_iterator end() const { return m_hash_vector.end(); }

    void clear() { m_hash_vector.clear(); }
    
  private:
    unsigned int m_hash_size;
    HashMap m_hash_vector;
    osg::Vec3 m_dimension;
    osg::Vec3 m_center;
  };

  template <class T>  
  inline void HashedGrid<T>::insert(const osg::Vec3& p, T data)
  {
    int x,y,z;
    calcCell(p,x,y,z);

    unsigned long idx = computeHashBucketIndex(x,y,z);
    HashMap::iterator it = m_hash_vector.find(idx);
    
    if (it == m_hash_vector.end()) {
      HashedDataVector *v = new HashedDataVector;
      v->insert(data);
      m_hash_vector.insert(std::make_pair(idx, v));
    }
    else
      it->second->insert(data);

    //std::cerr << "Added: " << idx << ": " << x << ", " << y << ", " << z << std::endl;
  }


  template <class T>  
    inline unsigned long HashedGrid<T>::computeHashBucketIndex (
      const  int & x, 
      const  int & y, 
      const  int & z) const
    {
      const l= m_hash_size*m_hash_size*m_hash_size;
      const unsigned long  h1 = 0x8dba6b343; //Large primes
      const unsigned long h2 = 0xd8163841;
      const unsigned long h3 = 0xcb1ab31f;
      unsigned long n = h1*x + h2*y + h3*z;
      n = n%l;
      if (n<0) n+=l;
      return n;
    }
  
}

#endif
