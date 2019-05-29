#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/SemanticOcTree.h>
#include "testing.h"

#define MAP_RESOLUTION 0.05
#define NUM_LABELS 10
#define NUM_EXTRAINFO 1

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << "   test.ply (point cloud file)"
    << "   color.txt (label to color mapping) \n\n";

  exit(1);
}

void print_query_info(point3d query, SemanticOcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    cout << "semantics probability is: " << node->getSemantics() << endl;
  }
  else
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}

std::vector<std::vector<float> > get_hard_labels(int num_labels) {
  std::vector<std::vector<float> > labels;
  labels.resize(num_labels);
  for (int i = 0; i < num_labels; i++) {
    for (int j = 0; j < num_labels; j++) {
      if (i == j)
        labels[i].push_back(1);
      else
        labels[i].push_back(0);
    }
  }
  return labels;
}

Pointcloud* labels_to_color (std::string colorfile) {
  Pointcloud* color = new Pointcloud();
  std::ifstream infile(colorfile.c_str());
  while (infile) {
    color->read(infile);
  }
  return color;
}


int main(int argc, char** argv) {
  if (argc < 3){
    printUsage(argv[0]);
  }

  std::string pointfile = std::string(argv[1]);
  std::string colorfile = std::string(argv[2]);

  // prepare labels and color vector
  std::vector<std::vector<float> > labels = get_hard_labels(NUM_LABELS);
  Pointcloud* color = labels_to_color(colorfile);

  // read point cloud with label from text file
  Pointcloud* cloud = new Pointcloud();
  std::ifstream infile(pointfile.c_str());
  while (infile) {
    cloud->readExtraInfo(infile, NUM_EXTRAINFO);
  }
  
  // set scan pose
  pose6d origin(0, 0, 0, 0, 0, 0);
  
  // insert into OcTree
  SemanticOcTree* tree = new SemanticOcTree(MAP_RESOLUTION);
  tree->insertPointCloud(*cloud, origin.trans());

  // fuse semantics
  for (int i = 0; i < (int)cloud->size(); ++i) {
    const point3d& query = (*cloud)[i];
    std::vector<float> extra_info = cloud->getExtraInfo(i);
    if(int(extra_info[NUM_EXTRAINFO-1]) == 1)
      continue;
    SemanticOcTreeNode* n = tree->search(query);
    tree->averageNodeSemantics(n, labels[int(extra_info[NUM_EXTRAINFO-1]) - 1]);
    //print_query_info(query, n);
  }

  // visualization
  for (SemanticOcTree::iterator it = tree->begin(); it != tree->end(); ++it) {
    if ( it->isSemanticsSet() ) {
      SemanticOcTreeNode::Semantics s = it->getSemantics();
      for (int i = 0; i < NUM_LABELS; i++) {
        if (s.label.size() && s.label[i] > 0.5) {
          it->setColor((*color)[i](0), (*color)[i](1), (*color)[i](2));
        }
      }
    }
  }
  tree->write("semantic_color_scan.ot");

  // output labels
  for (int i = 0; i < (int)cloud->size(); ++i) {
    const point3d& query = (*cloud)[i];
    SemanticOcTreeNode* n = tree->search(query);
    if ( n->isSemanticsSet() )
      cout << n->getSemantics() << endl;
    else
      cout << labels[0] << endl;
  }

  //cout << "Test done." << endl;
  exit(0);

}
