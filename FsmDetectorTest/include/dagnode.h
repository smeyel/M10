#ifndef __DAGNODE_H
#define __DAGNODE_H

#include <vector>

using namespace std;

namespace smeyel
{
	class DagNode
	{
	private:
		static int nextFreeNodeID;

		// Private, as setter-getter has many functions!
		int status;	// Auxiliary status value used by the applications

	protected:
		int nodeID;
		vector<DagNode *> parents;	// may contain NULL-s
		vector<DagNode *> children;	// may contain NULL-s

		// --- parent handling
		int getParentIdx(DagNode *parent)
		{
			for(int i=0; i<parents.size(); i++)
			{
				if (parents[i]==parent)
				{
					return i;
				}
			}
			return -1;
		}

	public:
		DagNode()
		{
			nodeID=nextFreeNodeID++;
		}

		// --- parent handling
		/** Adds given node to the parent list, if not yet present. */
		void addParent(DagNode *parent)	// called by setChild
		{
			int idx = getParentIdx(parent);
			if (idx==-1)
			{
				parents.push_back(parent);
			}
		}

		/** Remove node from list of parents, if present.
			Typically used by setChild().
		*/
		void removeParent(DagNode *parent)	// called by setChild
		{
			int idx = getParentIdx(parent);
			if (idx==-1)
			{
				parents[idx]=NULL;
			}
		}

		/** Returns parent pointer of given index. Used for enumerations.
			TODO: make iterator instead!
		*/
		DagNode *getParent(unsigned int idx)
		{
			if (idx < parents.size())
			{
				return parents[idx];
			}
			return NULL;
		}

		int getParentNum()
		{
			return parents.size();
		}

		// --- child handling
		/** Sets the child for given idx.
			Fixes parent pointers of old and new children.
			@param idx	Idx of child to set.
			@param child	Pointer to new child node. May be NULL.
		*/
		void setChild(unsigned idx, DagNode *child)
		{
			if (idx >= children.size())
			{
				children.reserve(idx+1);	// By default NULL lesz?
			}
			// Had a previous value?
			DagNode *oldChild = children[idx];
			if (oldChild!=NULL)
			{
				oldChild->removeParent(this);
			}
			children[idx] = child;
			if (child != NULL)
				child->addParent(this);
		}

		/** Get the child corresponding to idx. May be asked to create on demand. Otherwise, NULL may be retured. */
		DagNode *getChild(unsigned int idx, bool createOnDemand=false)
		{
			DagNode *result = NULL;
			if (idx < children.size())
			{
				result = children[idx];
			}
			if (result==NULL && createOnDemand)
			{
				result = new DagNode();
				setChild(idx,result);
			}
			return result;
		}

		/** Size of children array.
			@warning Children array may contain NULL-s!
		*/
		int getChildNum()
		{
			return children.size();
		}

		// --- status handling
		int setStatus(int newStatus)
		{
			status = newStatus;
		}

		int getStatus()
		{
			return status;
		}

		// --- advanced functions: rerouting of pointers
		void replaceChild(DagNode *oldNode, DagNode *newNode)
		{
			// replace current children pointers
			for(int i=0; i<getChildNum(); i++)
			{
				DagNode *child = getChild(i,false);
				if (child == oldNode)
				{
					setChild(i,newNode);	// fixes parent pointers to this node
				}
			}
		}

		/** All references to the current node are redirected to newNode. */
		void replaceNode(DagNode *newNode)
		{
			for(int i=0; i<getParentNum(); i++)
			{
				DagNode *parent = getParent(i);
				if (parent != NULL)
				{
					parent->replaceChild(this, newNode);	// This modifies the result of getParent(i) (!)
				}
			}
		}

		/** Forward recursive check subgraph status homogenity */
		bool isForwardSubgraphStatusHomogene()
		{
			for(int i=0; i<getChildNum(); i++)
			{
				DagNode *child = getChild(i,false);
				if (child != NULL)
				{
					int childStatus = child->getStatus();
					if (childStatus != status)
					{
						return false;
					}
					int childResult = child->isForwardSubgraphStatusHomogene();
					if (!childResult)
					{
						return false;
					}
				}
			}
			return true;
		}

		/** Cut subgraph if homogeneous and has same status. (Forward recursive if inhomogeneous.)
			@return True if this node may be also deleted, as it has the same status as the parent, and the whole subgraph as well.
		*/
		bool cut(int parentStatus)
		{
			//cout << "--- Cut@" << this->nodeID << " parentStatus=" << parentStatus << " status=" << status << endl;

			// Recursive check for all children
			bool allRemoved = true;
			for(int i=0; i<children.size(); i++)
			{
				DagNode *child = getChild(i,false);
				if (child)
				{
					// Children are checked with this->status in every case.
					bool result = child->cut(this->status);

					if (!result)
					{
						//cout << "Cut@" << this->nodeID << " child " << i << ": keep" << endl;
						allRemoved = false;	// Could not remove all children
					}
					else
					{
						//cout << "Cut@" << this->nodeID << " child " << i << ": DEL" << endl;
						// Redirect all pointers to this child to this node.
						child->replaceNode(this);	// This may add further parents to this->parenst (!)
						delete child;
						children[i] = NULL;
					}
				}
			}

			// This node can be removed if
			//	- all children are removed, and
			//	- the auxScore is the same as of the parent (caller)
			if (allRemoved && (this->status == parentStatus))
			{
				//cout << "Cut@" << this->nodeID << " removeable" << endl;
				return true;
			}
			//cout << "Cut@" << this->nodeID << " not removeable" << endl;
			return false;
		}
	};

	int DagNode::nextFreeNodeID = 0;

	class CounterDagNode : public DagNode
	{
	protected:
		int posCount;
		int negCount;
	public:
		CounterDagNode() : DagNode()
		{
			posCount = 0;
			negCount = 0;
		}

		void incrementCounter(bool isPositive)
		{
			if (isPositive)
				posCount++;
			else
				negCount++;
		}

		// Assumes that all nodes have type ClassifierDagNode!
		void incrementCounterForInputSequence(unsigned int *inputSequence, unsigned int valueNumberToUse, bool isPositive, bool createChildrenOnDemand=false)
		{
			incrementCounter(isPositive);
			CounterDagNode *child = (CounterDagNode *)getChild(*inputSequence,createChildrenOnDemand);
			if (child!=NULL && valueNumberToUse>1)
			{
				child->incrementCounterForInputSequence(inputSequence+1,valueNumberToUse-1,isPositive,createChildrenOnDemand);
			}
		}
	};

	class ClassifierDagNode : public CounterDagNode
	{
	private:
		float precision;
	public:
		static const int STATUS_LOWPRECISION	= 0;
		static const int STATUS_HIGHPRECISION	= 1;

		ClassifierDagNode() : CounterDagNode()
		{
			precision=0.0F;
		}

		/** Forward recursive calculation of precisions and setting status for every node
			to STATUS_LOWPRECISION or STATUS_HIGHPRECISION.
		*/
		void calculatePrecisions(int rootPosCount, int rootNegCount, float minPrecision)
		{
			float posRate = (float)posCount / (float)rootPosCount;
			float negRate = (float)negCount / (float)rootNegCount;
			precision = posRate / (posRate + negRate);	// TODO: is this calculation OK?

			setStatus((precision>=minPrecision)?STATUS_HIGHPRECISION:STATUS_LOWPRECISION);

			for(int i=0; i<children.size(); i++)
			{
				ClassifierDagNode *child = (ClassifierDagNode *)getChild(i,false);
				if (child)
				{
					child->calculatePrecisions(rootPosCount,rootNegCount, minPrecision);
				}
			}
		}
	};
}

#endif
