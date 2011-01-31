/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Sensor Library
* Copyright (C) 2006 VRlab, Umeå University
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
*/

#include <string.h>

#include "osgSensor/Visitors.h"

using namespace osgSensor;


osg::Node *FindVisitor::find(const std::string& name, osg::Node *root, StringMatchMode mode) { 
  m_mode = mode;
  
  // Bug now fixed: Reset result before searching for a node
  m_result = 0L;
  
  if (!root)
    return 0L;
    
  m_name = name; 
  apply(*root); 
  return m_result; 
}

void trim(std::string& str) {
  unsigned int pos=str.length();
  
  while(pos && (!isprint(str[pos-1]) || isspace(str[pos]) || str[pos-1] == ' '))
    pos--;
  
  if (pos)
    str = str.substr(0,pos);
}
  
void FindVisitor::apply(osg::Node& node)
{
  std::string name = node.getName();
  trim(name);
  
  
  if (m_mode == Equal && name.length() == m_name.length() && !strncmp(name.c_str(), m_name.c_str(), name.length())) {//); == m_name) {
    m_result = &node;
    return;
  }
  else if (m_mode == Substring && name.find(m_name) != std::string::npos) {
    m_result = &node;
    return;
  }
  else if (m_mode == StartsWith && name.substr(0, m_name.length()) == m_name) {
    m_result = &node;
    return;
  }
  traverse(node);
}
  

osg::Node *FindDescriptionVisitor::find(const std::string& description, osg::Node *root, 
                                        StringMatchMode mode, int max_depth) { 
  
  m_mode = mode;
  m_max_depth = max_depth;

  m_curr_depth=0;

  m_result = 0L;
  if (!root)
    return 0L;
    
  m_description = description; 

  apply(*root); 
  return m_result; 
}

  
void FindDescriptionVisitor::apply(osg::Node& node)
{
  m_curr_depth++;
  unsigned int n = node.getNumDescriptions();

  if (!n) {
    if (m_max_depth == -1 || m_curr_depth < m_max_depth)
      traverse(node);
    return;
  }

  for (unsigned int i=0; i < n; i++) {
    std::string description = node.getDescription(i);

    if (m_mode == Equal && description.substr(0,m_description.length()) == m_description) {
      m_result = &node;
      return;
    }
    else if (m_mode == Substring && description.find(m_description) != std::string::npos) {
      m_result = &node;
      return;
    }
    else if (m_mode == StartsWith && description.substr(0, m_description.length()) == m_description) {
      m_result = &node;
      return;
    }
  }

  if (m_max_depth == -1 || m_curr_depth < m_max_depth)
    traverse(node);
}
  

const osg::Matrix& NodePathCollectMatrix::collect(const osg::NodePath& node_path, 
                                                  osg::Node *stop_node)
{
  m_nodepath = &node_path;
  m_stop_node = stop_node;

  m_matrix.makeIdentity();
  
  // Empty list?
  if (!m_nodepath->size())
    return m_matrix;

  m_iterator = node_path.rbegin();

  if (m_iterator == m_nodepath->rend())
    return m_matrix;

  apply(*m_iterator);

  return m_matrix;
}

void NodePathCollectMatrix::apply(const osg::MatrixTransform& node)
{ 
  // Stop when we have reached the m_stop_node
  if (m_stop_node == &node)
    return;

  // The matrix is not used for the stop node
  m_matrix *= node.getMatrix();

  // Find next node in NodePath
  traverse();
}

void NodePathCollectMatrix::apply(const osg::Node& node)
{ 
  // Stop when we have reached the m_stop_node
  if (m_stop_node == &node)
    return;

  traverse();
}

// Find next node in NodePath
void NodePathCollectMatrix::traverse() {
  m_iterator++;
  if (m_iterator == m_nodepath->rend())
    return;
  
  apply(*m_iterator);
}

void NodePathCollectMatrix::apply(const osg::Node *pNode)
{
  const osg::MatrixTransform *mt = dynamic_cast<const osg::MatrixTransform *>(pNode);
  if (mt)
    apply(*mt);
  else
    apply(*pNode);
}
