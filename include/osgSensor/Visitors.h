//C++ header - vrutils
// $Id: Visitors.h,v 1.6 2006/03/27 06:47:51 andersb Exp $

#ifndef __Visitor_h__
#define __Visitor_h__


#include <osg/NodeVisitor>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <assert.h>
#include <iostream>

#include "osgSensor/export.h"

namespace osgSensor {


class OSGSENSOR_EXPORT FindVisitor : public osg::NodeVisitor{

public:

  enum StringMatchMode { StartsWith, Substring, Equal };

  FindVisitor( const std::string& name, osg::NodeVisitor::TraversalMode tm ) : NodeVisitor(tm), m_result(0), m_name(name) {}
  FindVisitor(  ) : NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_result(0) {}

  virtual ~FindVisitor(){}
  
  /// Specify the name of the node we will search for
  void setName(const std::string& name) { m_name = name; }

  /// Specify the mode for matching the name of the node
  void setStringMatchMode(StringMatchMode mode) { m_mode = mode; }

  /// Get the result of the visitor
  osg::Node *getResult() { return m_result; }

  ///
  virtual osg::Node *find(const std::string& name, osg::Node *root, StringMatchMode mode=Equal); 

protected:


  virtual void apply(osg::Node& node);

private:
  osg::Node *m_result;
  std::string m_name;
  StringMatchMode m_mode;
};


class OSGSENSOR_EXPORT PrintVisitor : public osg::NodeVisitor
{

   public:
   
        PrintVisitor():NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN)
        {
            _indent = 0;
            _step = 2;
        }
        
        inline void moveIn() { _indent += _step; }
        inline void moveOut() { _indent -= _step; }
        inline void writeIndent() 
        {
            for(int i=0;i<_indent;++i) std::cout << " ";
        }
        virtual void apply(osg::Node* node) { apply(*node); }                
        virtual void apply(osg::Node& node)
        {
            moveIn();
            writeIndent(); 
            
            if (!node.getName().length())
              std::cerr << "_";
            else
              std::cerr << node.getName();
              
            std::cerr << ": " << node.className() <<std::endl;


            traverse(node);
            moveOut();
        }


   protected:
    
        int _indent;
        int _step;
};


/// Concatenates matrix traversing all parents
class OSGSENSOR_EXPORT CollectTransforms : public osg::NodeVisitor
{
public:

  /// Constructor
  CollectTransforms() : NodeVisitor(NodeVisitor::TRAVERSE_PARENTS) { m_matrix.makeIdentity(); }

  

  /// Get matrix
  const osg::Matrix& getMatrix() const { return m_matrix; }

  const osg::Matrix& collect(osg::Node& node, osg::Node *stop_node=0L) { 
    m_stop_node=stop_node; m_matrix.makeIdentity(); apply(node); return m_matrix; 
  }
  
  const osg::Matrix& collect(osg::MatrixTransform& node, osg::Node *stop_node=0L) { 
    m_stop_node = stop_node;
    m_matrix.makeIdentity(); apply(node); return m_matrix; 
  }


protected:

  /// Apply
  virtual void apply(osg::Node& node) { 
    // Stop when we have reached the m_stop_node
    if (m_stop_node == &node)
      return;

    traverse(node);
  }

  /// Apply
  virtual void apply(osg::MatrixTransform& node) { 

    // Stop when we have reached the m_stop_node
    if (m_stop_node == &node)
      return;

    // The matrix is not used for the stop node
    m_matrix *= node.getMatrix();

    traverse(node);
  }

  /// Apply
  virtual void apply(osg::PositionAttitudeTransform& node) { 

	  // Stop when we have reached the m_stop_node
	  if (m_stop_node == &node)
		  return;

	  osg::Matrix m;
	  node.computeLocalToWorldMatrix(m,NULL);
	  // The matrix is not used for the stop node
	  m_matrix *= m;

	  traverse(node);
  }

  /// Matrix
  osg::Matrix m_matrix;
  osg::Node *m_stop_node;

};



///
class OSGSENSOR_EXPORT FindDescriptionVisitor : public osg::NodeVisitor{

public:

  ///
  enum StringMatchMode { StartsWith, Substring, Equal };

  ///
  FindDescriptionVisitor(TraversalMode tm=TRAVERSE_ALL_CHILDREN, int max_depth=-1 ) : 
    NodeVisitor(tm), 
    m_result(0), m_max_depth(max_depth), m_curr_depth(0) {}

  ///
  virtual ~FindDescriptionVisitor(){}

  ///
  virtual osg::Node *find(const std::string& description, osg::Node *root, StringMatchMode mode=Equal, int max_depth=-1); 

protected:

  ///
  virtual void apply(osg::Node& node);

private:
  osg::Node *m_result;
  std::string m_description;
  StringMatchMode m_mode;
  int m_curr_depth;
  int m_max_depth;
};

/// Concatenates matrix traversing all parents
class OSGSENSOR_EXPORT NodePathCollectMatrix
{
public:

  /// Constructor
  NodePathCollectMatrix()  { m_matrix.makeIdentity(); }
  
  /// Destructor
  virtual ~NodePathCollectMatrix() {}
  const osg::Matrix& collect(const osg::NodePath& node_path, osg::Node *stop_node=0L);

  /// Get matrix
  const osg::Matrix& getMatrix() const { return m_matrix; }

protected:

  void apply(const osg::Node *pNode);

  void traverse();

  /// Apply
  virtual void apply(const osg::Node& node); 

  /// Apply
  virtual void apply(const osg::MatrixTransform& node); 
  /// Matrix
  osg::Matrix m_matrix;
  const osg::Node *m_stop_node;
  osg::NodePath::const_reverse_iterator m_iterator;
  const osg::NodePath *m_nodepath;
};




}; // namespace osgSensor
#endif

