#include "ClusterCMA.h"
#include "common/app_cppcommon.h"

ClusterCMA::ClusterCMA()
    : root(NULL)
{
}

ClusterCMA::~ClusterCMA() {
}

void ClusterCMA::setCMA(const CMA2& cma) {
    if (root) {
        delete root;
    }

    CMA2::resetID();
    root = new CMA2();
    (*root) = cma;
    
    // orig = cma;
    // CMAs.push_back(cma);
}

void ClusterCMA::calculateClusterIndex(int mu) {
    root->resetIndFlagsSubtree();
    std::vector<CMA2*> leafNodes;
    root->collectLeafNodes(leafNodes);
    LOG(INFO) << "# of leaf nodes = " << leafNodes.size();
    CHECK_LT(0, leafNodes.size());
    
    indexToCluster.clear();
    while(indexToCluster.size() < mu) {
        FOREACH(CMA2* leafNode, leafNodes) {
            for (int j = 0; j < leafNode->nCluSize; j++) {
                indexToCluster.push_back(leafNode);
            }
        }
    }
}



void ClusterCMA::create(Individual &o, int index) {
    CMA2* cma = indexToCluster[index];
    cout << FUNCTION_NAME() << " : " << index << " <-- cluster " << cma->name << endl;
    cma->create(o);
    cma->setIndFlagsToRoot(index);
}

// Eigen::VectorXd ClusterCMA::toEigen(IndividualT<double>& ind) {
//     int n = ind[0].size();
//     Eigen::VectorXd ret(n);
//     for (int i = 0; i < n; i++) {
//         ret(i) = ind[0][i];
//     }
//     return ret;
// }

// Eigen::VectorXd ClusterCMA::mean(PopulationT<double> &p) {
//     int n = p[0][0].size();
//     Eigen::VectorXd ret = Eigen::VectorXd::Zero(n);
//     for (int i = 0; i < p.size(); i++) {
//         ret += toEigen(p[i]);
//     }
//     ret /= (double)p.size();
//     return ret;
// }

// CMA2& ClusterCMA::findParent(PopulationT<double> &p) {
//     Eigen::VectorXd m = mean(p);

//     int ret = -1;
//     double minDist = 999999.9;
//     for (int i = 0; i < CMAs.size(); i++) {
//         CMA2& cma = CMAs[i];
//         Eigen::VectorXd cen = cma.center();
//         double d = (cen - m).norm();
//         if (d < minDist) {
//             minDist = d;
//             ret = i;
//         }
//     }
//     cout << "findParent.return = " << ret << endl;
//     return CMAs[ret];
// }


void ClusterCMA::updateStrategyParameters(mpisolver::Rclustering& clust,
                                          PopulationT<double> &p,
                                          const PopulationT<double> &offspring)
{
    name.clear();
    name.resize( p.size() );
    cout << "==== the previous tree ====" << endl;
    root->setCluSizeSubtree(0);
    root->printIndFlagsSubtree( offspring.size() );

    for (int i = 0; i < clust.numGroups(); i++) {
        PopulationT<double> my;
        for (int j = 0; j < clust.cls.size(); j++) {
            if (clust.cls[j] != i) {
                continue;
            }
            my.append( p[j] );
        }
        long long indflags = toIndflags(my, offspring);
        cout << "Cluster " << i << " : " << endl;
        CMA2::printIndFlags(offspring.size(), indflags);
        CMA2* parent = root->findTheMostBottomNode(indflags);
        CHECK_NOTNULL(parent);

        CMA2* me = new CMA2(*parent);
        CHECK(me->parent == NULL);
        me->name = parent->name + me->name;
        me->indflags = indflags;
        me->nCluSize = my.size();
        // (*me) = (*parent);
        parent->addChild(me);

        cout << " >> parent = " << parent->name << endl;

        for (int j = 0; j < clust.cls.size(); j++) {
            if (clust.cls[j] != i) {
                continue;
            }
            name[j] = me->name;
        }
    }
    long long parentflags = toIndflags(p, offspring);
    root->selectParentsSubtree(parentflags);
    root->updateCMASubtree(offspring);
    cout << "==== the current tree ====" << endl;
    root->printIndFlagsSubtree( offspring.size() );

    
    // for (int i = 0; i < p.size(); i++) {
    //     IndividualT<double>& ind = p[i];
    //     int offspringIndex = findIndex(ind, offspring);
    //     cout << i << "  <---->   " << offspringIndex << endl;
    // }
    // // CMAs.clear();
    // std::vector<CMA2> CMAs_next;

    
    // for (int i = 0; i < clust.numGroups(); i++) {
    //     PopulationT<double> myParents;
    //     for (int j = 0; j < clust.cls.size(); j++) {
    //         if (clust.cls[j] != i) {
    //             continue;
    //         }
    //         myParents.append( p[j] );
 
    //     }

    //     cout << "call findParent for group " << i << endl;
    //     CMA2 next = findParent(myParents);
        
    //     // CMA2 next = orig;
    //     next.updateStrategyParameters( myParents );
    //     CMAs_next.push_back(next);
    // }
    // CMAs = CMAs_next;
    
    // orig.updateStrategyParameters(p);
}

int ClusterCMA::findIndex(const IndividualT<double>& o, const PopulationT<double>& population) {
    int n = o[0].size();
    int matchedIndex = -1;
    for (int i = 0; i < population.size(); i++) {
        const IndividualT<double>& rhs = population[i];
        bool isEqual = true;
        for (int j = 0; j < n; j++) {
            if (o[0][j] != rhs[0][j]) {
                isEqual = false;
                break;
            }
        }
        if (isEqual) {
            matchedIndex = i;
            break;
        }
    }
    CHECK_NE(matchedIndex, -1) << " couldn't not find the individual index in the population";
    return matchedIndex;
}

long long ClusterCMA::toIndflags(const PopulationT<double> &my,
                                 const PopulationT<double> &population) {
    long long indflags = 0;
    for (int i = 0; i < my.size(); i++) {
        int id = findIndex(my[i], population);
        indflags = ( indflags | ( (long long)1 << id ) );
    }
    return indflags;
}


std::string ClusterCMA::toRstr() const {
    return root->toRstrSubtree();
    // std::stringstream sout;
    // for (int i = 0; i < CMAs.size(); i++) {
    //     const CMA2& cma = CMAs[i];
    //     sout << cma.toRstr() << endl;
    // }
    // return sout.str();
}

void ClusterCMA::writeCSV(const char* const filename) const {
    using std::endl;
    std::ofstream fout(filename);
    fout << toRstr() << endl;
    fout.close();
}
