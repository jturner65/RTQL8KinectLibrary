#include "widget_opt_explorer.h"

#include <sstream>
#include "boost/lexical_cast.hpp"
#include "utils/Paths.h"
#include "common/app_cppcommon.h"
// Local headers
#include "window.h"

namespace gui {
    WidgetOptExplorer::WidgetOptExplorer(Window* _parent)
        : QTreeView(_parent)
        , MEMBER_INIT_NULL(model)
    {
        init();
        setFixedSize(360, 480);

    }

    void WidgetOptExplorer::init() {
        QStandardItemModel* m = new QStandardItemModel();
        QStringList headers;
        // headers << "iter" << "index" << "gene" << "value";
        headers << "tag" << "gene" << "value";
        m->setHorizontalHeaderLabels(headers);
        set_model(m);


        this->setModel(model());

        this->setColumnWidth(0, 80);
        this->setColumnWidth(1, 180);
        this->expandAll();

        reload();
    }

    void WidgetOptExplorer::reload() {
        model()->clear();

        QStringList headers;
        // headers << "iter" << "index" << "gene" << "value";
        headers << "tag" << "gene" << "value";
        model()->setHorizontalHeaderLabels(headers);

        loadXML(RTQL8_DATA_PATH"xml/result.xml");
        cout << FUNCTION_NAME() << " OK" << endl;
    }

    bool WidgetOptExplorer::hasSelection() {
        QModelIndexList list = selectionModel()->selectedIndexes();
        if (list.size() > 0) {
            return true;
        } else {
            return false;
        }
    }

    int WidgetOptExplorer::selectedRow() {
        QModelIndexList list = selectionModel()->selectedIndexes();
        if (list.size() > 0) {
            QModelIndex index = list.at(1);
            return index.row();
        } else {
            return -1;
        }
    }

    Eigen::VectorXd WidgetOptExplorer::selectedGene() {
        Eigen::VectorXd ret(1);
        ret << 0.0;
        
        QModelIndexList list = selectionModel()->selectedIndexes();
        if (list.size() > 0) {
            QModelIndex index = list.at(1);
            QString text = model()->itemFromIndex(index)->text();
            std::stringstream sout;
            sout << text.toStdString();
            sout >> IO(ret);
        }
        return ret;

    }

    void WidgetOptExplorer::loadXML(const char* const filename) {
        cout << FUNCTION_NAME() << " : " << filename << endl;
        rapidxml::file<> xmlFile(filename);
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        rapidxml::xml_node<>* child;
        if ( (child = doc.first_node("result")) ) {
            readResult(child);
            // readOptimization(child);
        }        
        cout << FUNCTION_NAME() << " OK" << endl;
    }

    void WidgetOptExplorer::readResult(rapidxml::xml_node<>* node) {
        for (rapidxml::xml_node<> *child = node->first_node("sample");
             child; child = child->next_sibling("sample")) {
            readSample(child);
        }
    }

    // void WidgetOptExplorer::readIteration(rapidxml::xml_node<>* node) {
    //     int iter = -1;
    //     rapidxml::xml_attribute<>* attr = node->first_attribute("iter");
    //     if (attr) {
    //         iter = boost::lexical_cast<int>(attr->value());
    //     }
    //     cout << FUNCTION_NAME() << " : " << iter << " OK" << endl;

    //     QList<QStandardItem*> row;
    //     QStandardItem* item = new QStandardItem(QString("iter %0").arg(iter));
    //     row << item;
    //     model()->appendRow(row);
        
    //     for (rapidxml::xml_node<> *child = node->first_node("parent");
    //          child; child = child->next_sibling("parent")) {
    //         readParent(child, item, iter);
    //     }
        
        
    // }

    void WidgetOptExplorer::readSample(rapidxml::xml_node<>* node) {
                                       // QStandardItem* item,
                                       // int iter) {
        rapidxml::xml_attribute<>* attr = NULL;

        // int index;
        // if ( (attr = node->first_attribute("index")) ) {
        //     index = boost::lexical_cast<int>(attr->value());
        // }

        std::string tag;
        if ( (attr = node->first_attribute("tag")) ) {
            tag = boost::lexical_cast<std::string>(attr->value());
        }


        std::string gene;
        if ( (attr = node->first_attribute("gene")) ) {
            gene = boost::lexical_cast<std::string>(attr->value());
        }

        double value;
        if ( (attr = node->first_attribute("value")) ) {
            value = boost::lexical_cast<double>(attr->value());
        }
        cout << FUNCTION_NAME() << " : " << gene << ", " << value << endl;

        QList<QStandardItem*> row;
        row << new QStandardItem(QString(tag.c_str()));
        row << new QStandardItem(QString(gene.c_str()));
        row << new QStandardItem(QString("%0").arg(value));

        // item->appendRow(row);
        model()->appendRow(row);

    }




} // namespace gui
