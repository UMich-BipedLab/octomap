#include <octomap/SemanticOcTree.h>

namespace octomap {


  // node implementation  --------------------------------------
  std::ostream& SemanticOcTreeNode::writeData(std::ostream &s) const {
	s.write((const char*) &value, sizeof(value)); // occupancy
    s.write((const char*) &color, sizeof(Color)); // color
	return s;
  }

  std::istream& SemanticOcTreeNode::readData(std::istream &s) {
	s.read((char*) &value, sizeof(value)); // occupancy
	s.read((char*) &color, sizeof(Color)); // color
	return s;
  }

  SemanticOcTreeNode::Color SemanticOcTreeNode::getAverageChildColor(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> & label2color) const {
    if (label2color.size() == 0) {
      int mr = 0;
      int mg = 0;
      int mb = 0;
      int c = 0;

      if (children != NULL){
        for (int i=0; i<8; i++) {
          SemanticOcTreeNode* child = static_cast<SemanticOcTreeNode*>(children[i]);
        
          if (child != NULL && child->isColorSet()) {
            mr += child->getColor().r;
            mg += child->getColor().g;
            mb += child->getColor().b;
            ++c;
          }
        }
      }
    
      if (c > 0) {
        mr /= c;
        mg /= c;
        mb /= c;
        return Color((uint8_t) mr, (uint8_t) mg, (uint8_t) mb);
      }
      else { // no child had a color other than white
        return Color(255, 255, 255);
      }
    } else {
      // use provided color map
      auto & distribution = semantics.label;
      
      if (distribution.size() == 0 ) {
        return Color(255, 255, 255);
      }
      int max_prob_class = std::distance(distribution.begin(),
                                         std::max_element(distribution.begin(), distribution.end()));

      auto color_max = label2color.at(max_prob_class);

      return Color(std::get<0>(color_max),
                   std::get<1>(color_max),
                   std::get<2>(color_max));

    }
  }
 SemanticOcTreeNode::Color SemanticOcTreeNode::getAverageChildColor() const {

   int mr = 0;
   int mg = 0;
   int mb = 0;
   int c = 0;

   if (children != NULL){
     for (int i=0; i<8; i++) {
       SemanticOcTreeNode* child = static_cast<SemanticOcTreeNode*>(children[i]);
        
       if (child != NULL && child->isColorSet()) {
         mr += child->getColor().r;
         mg += child->getColor().g;
         mb += child->getColor().b;
         ++c;
       }
     }
   }
    
   if (c > 0) {
     mr /= c;
     mg /= c;
     mb /= c;
     return Color((uint8_t) mr, (uint8_t) mg, (uint8_t) mb);
   }
   else { // no child had a color other than white
     return Color(255, 255, 255);
   }

  }

  SemanticOcTreeNode::Semantics SemanticOcTreeNode::getAverageChildSemantics() const {
    std::vector<float> mlabel;
    int c = 0;

    if (children != NULL) {
      for (int i=0; i<8; i++) {
        SemanticOcTreeNode* child = static_cast<SemanticOcTreeNode*>(children[i]);

        if (child != NULL && child->isSemanticsSet()) {
          std::vector<float> clabel = child->getSemantics().label;
          if (mlabel.empty())
            mlabel.reserve(clabel.size());
          else if (mlabel.size() < clabel.size())
            mlabel.resize(clabel.size());

          for (int l=0; l<(int)clabel.size(); l++) {
            mlabel[l] += clabel[l];
          }
          ++c;
        }
      }
    }

    if (c > 0) {
      for (int l=0; l<(int)mlabel.size(); l++) {
        mlabel[l] /= c;
      }
      return Semantics(mlabel);
    }
    else { // no child had a semantics other than empty
      return Semantics();
    }
  }

  void SemanticOcTreeNode::updateColorChildren() {
	color = getAverageChildColor();
  }

  void SemanticOcTreeNode::updateColorChildren(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> & label2color_map) {
	color = getAverageChildColor(label2color_map);
  }

  
  void SemanticOcTreeNode::updateSemanticsChildren(){
	semantics = getAverageChildSemantics();
  }

  void SemanticOcTreeNode::normalizeSemantics() {
    float sum = 0;
    for (int i = 0; i < (int)semantics.label.size(); i++)
      sum += semantics.label[i];
    if (sum > 0) {
      for (int i = 0; i < (int)semantics.label.size(); i++)
        semantics.label[i] = semantics.label[i]/sum;
    } else {
      for (int i = 0; i < (int)semantics.label.size(); i++)
        semantics.label[i] = 1.0  / semantics.label.size();
      
    }
    
  }


  // tree implementation  --------------------------------------
  SemanticOcTree::SemanticOcTree(double resolution)
  : OccupancyOcTreeBase<SemanticOcTreeNode>(resolution) {
    semanticOcTreeMemberInit.ensureLinking();
  };

  SemanticOcTree::SemanticOcTree(double resolution,
                                 int num_classes,
                                 const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> & label2color_map)
    : OccupancyOcTreeBase<SemanticOcTreeNode>(resolution) ,
    label2color(label2color_map),
    num_class(num_classes)
  {
    semanticOcTreeMemberInit.ensureLinking();
    
  };

  
  void SemanticOcTree::averageNodeColor(SemanticOcTreeNode* n,
                                        uint8_t r,
                                        uint8_t g,
                                        uint8_t b) {
    if (n != 0) {
      if (n->isColorSet()) {
        SemanticOcTreeNode::Color prev_color = n->getColor();
        n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2);
      }
      else {
        n->setColor(r, g, b);
      }
    }
  }

  void SemanticOcTree::averageNodeSemantics(SemanticOcTreeNode* n,
                                            std::vector<float> &label) {
    if (n != 0) {
      if (n->isSemanticsSet()) {
        SemanticOcTreeNode::Semantics prev_semantics = n->getSemantics();
        if (prev_semantics.label.size() < label.size()) {
            prev_semantics.label.resize(label.size());
            for (int i = 0; i < label.size(); i++)
              prev_semantics.label[i] = 1.0 / label.size();
            
        }

        //std::cout<<"averageNodeSemantics: # of labels is "<<n->getSemantics().label.size()<<", the first one is "<<n->getSemantics().label[0]<<std::endl;

        
        //label[0] *= 0; // smaller weight for unlabeled data
        for (int i=0; i<(int)label.size(); i++) {
          prev_semantics.label[i] = (prev_semantics.label[i]*prev_semantics.count + label[i]);
          prev_semantics.label[i] = prev_semantics.label[i] / (prev_semantics.count + 1);
          //prev_semantics.label[i] = (prev_semantics.label[i] + label[i]) / 2;
        }
        
        n->setSemantics(prev_semantics);
        n->normalizeSemantics();
        n->addSemanticsCount();

      }
      else {
        n->setSemantics(label);
        n->normalizeSemantics();
        n->resetSemanticsCount();
      }
    }
    //std::cout<<"averageNodeSemantics: # of labels is "<<n->getSemantics().label.size()<<", the first one is "<<n->getSemantics().label[0]<<std::endl;
  }

  void SemanticOcTree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void SemanticOcTree::updateInnerOccupancyRecurs(SemanticOcTreeNode* node, unsigned int depth) {
    // only recurse and update for inner nodes:
    if (nodeHasChildren(node)){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (nodeChildExists(node, i)) {
            updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
      
      node->updateSemanticsChildren();

      if (label2color.size())
        node->updateColorChildren(label2color);
      else
        node->updateColorChildren();

    }
    else
      node->updateColorChildren(label2color);
  }

  std::ostream& operator<<(std::ostream& out,
      SemanticOcTreeNode::Color const& c) {
    return out << '(' << (unsigned int)c.r << ' ' << (unsigned int)c.g << ' ' << (unsigned int)c.b <<
                  ')';
  }

  std::ostream& operator<<(std::ostream& out,
      SemanticOcTreeNode::Semantics const& s) {
    for (unsigned i=0; i<s.label.size(); i++) {
      out << s.label[i] << ' ';
    }
    return out;
  }


  SemanticOcTree::StaticMemberInitializer SemanticOcTree::semanticOcTreeMemberInit;

} // end namespace

