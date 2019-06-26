#ifndef OCTOMAP_SEMANTIC_OCTREE_H
#define OCTOMAP_SEMANTIC_OCTREE_H


#include <iostream>
#include <unordered_map>
#include <tuple> // for color map
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>


namespace octomap {
  
  // forward declaraton for "friend"
  class SemanticOcTree;

  // node definition
  class SemanticOcTreeNode : public OcTreeNode {    
  public:
    friend class SemanticOcTree; // needs access to node children (inherited)
    
    class Color {
    public:
      Color() : r(255), g(255), b(255) {}
      Color(uint8_t _r, uint8_t _g, uint8_t _b) 
        : r(_r), g(_g), b(_b) {}
      inline bool operator== (const Color &other) const {
        return (r==other.r && g==other.g && b==other.b);
      }
      inline bool operator!= (const Color &other) const {
        return (r!=other.r || g!=other.g || b!=other.b);
      }
      uint8_t r, g, b;
    };

    class Semantics {
    public:
      Semantics() : label(), count(0) {}
      Semantics(int num_class)  {
        label.resize(num_class);
        for (int i = 0; i < num_class; i++ ) {
          label[i] = 1.0 / num_class;
        }
        count = 1;
      }
      Semantics(std::vector<float> _label)
        : label(_label), count(1) {}
      std::vector<float> label;
      unsigned int count;
    };

  public:
    SemanticOcTreeNode() : OcTreeNode() {}
    SemanticOcTreeNode(int num_class) : OcTreeNode() {
      semantics.label.resize(num_class);
      
    }

    SemanticOcTreeNode(const SemanticOcTreeNode& rhs) : OcTreeNode(rhs), color(rhs.color), semantics(rhs.semantics){}
        
    inline Color getColor() const { return color; }
    inline void  setColor(Color c) { this->color = c; }
    inline void  setColor(uint8_t r, uint8_t g, uint8_t b) {
      this->color = Color(r,g,b);
    }

    Color& getColor() { return color; }

    // has any color been integrated? (pure white is very unlikely...)
    inline bool isColorSet() const { 
      return ((color.r != 255) || (color.g != 255) || (color.b != 255)); 
    }

    inline Semantics getSemantics() const { return semantics; }
    
    inline int getSemanticLabel() const {
      int label = 0;
      float max = 0;
      for (unsigned int c = 0; c < semantics.label.size(); ++c) {
        if (semantics.label[c] > max) {
          label = c;
          max = semantics.label[c];
        }
      }
      return label;
    }
    
    inline void setSemantics(Semantics & s) {
      this->semantics = s;
    }
    inline void setSemantics(std::vector<float> & label) {
      this->semantics.label = label;
    }

    Semantics& getSemantics() { return semantics; }

    inline bool isSemanticsSet() const { 
      return (semantics.label.size() > 0);
    }

    inline void addSemanticsCount() { this->semantics.count++; }
    inline void resetSemanticsCount() { this->semantics.count = 1; }

    void updateColorChildren();
    void updateColorChildren(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> & label2color_map);
    void updateSemanticsChildren();
    void normalizeSemantics();

    SemanticOcTreeNode::Color getAverageChildColor() const;
    SemanticOcTreeNode::Color getAverageChildColor(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> & label2color_map) const;
    SemanticOcTreeNode::Semantics getAverageChildSemantics() const;

    // file I/O
    std::istream& readData(std::istream &s);
    std::ostream& writeData(std::ostream &s) const;

  protected:
    Color color;
    Semantics semantics;
  };


  // tree definition
  class SemanticOcTree : public OccupancyOcTreeBase <SemanticOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    SemanticOcTree(double resolution);
    /// Default constructor, sets resolution of leafs
    SemanticOcTree(double resolution, int num_classes,
                   const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> & label2color_map);

    void addColorMap(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> & label2color_map) {
      label2color = label2color_map;

    }
    

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    SemanticOcTree* create() const { return new SemanticOcTree(resolution); }

    std::string getTreeType() const { return "ColorOcTree"; }

    void averageNodeColor(SemanticOcTreeNode* n, uint8_t r, uint8_t g, uint8_t b);
    void averageNodeSemantics(SemanticOcTreeNode* n, std::vector<float> & label);

    // update inner nodes, sets color to average child color
    void updateInnerOccupancy();

  protected:
    void updateInnerOccupancyRecurs(SemanticOcTreeNode* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a 
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        SemanticOcTree* tree = new SemanticOcTree(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
      }

      /**
       * Dummy function to ensure that MSVC does not drop the
       * StaticMemberInitializer, causing this tree failing to register.
       * Needs to be called from the constructor of this octree.
       */
      void ensureLinking() {};
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer semanticOcTreeMemberInit;

    // use provided color map or not
    std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> label2color;
    int num_class;
  };

  //! user friendly output in format (r g b)
  std::ostream& operator<<(std::ostream& out,
                           SemanticOcTreeNode::Color const& c);

  std::ostream& operator<<(std::ostream& out,
                           SemanticOcTreeNode::Semantics const& s);

} // end namespace

#endif
