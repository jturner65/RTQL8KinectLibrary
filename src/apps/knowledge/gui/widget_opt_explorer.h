#ifndef GUI_WIDGET_OPT_EXPLORER_H
#define GUI_WIDGET_OPT_EXPLORER_H

#include <QWidget>
#include <QtGui>
#include <Eigen/Dense>
// RapidXML
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "rapidxml_utils.hpp"

#include "common/app_hppcommon.h"

namespace gui {
    class Window;
} // namespace gui

namespace gui {
    class WidgetOptExplorer : public QTreeView {
        Q_OBJECT;
    public:
        WidgetOptExplorer(Window* _parent);

        void init();
        void reload();

        bool hasSelection();
        int selectedRow();
        Eigen::VectorXd selectedGene();

        void loadXML(const char* const filename);
    protected:
        void readResult(rapidxml::xml_node<>* node);
        void readSample(rapidxml::xml_node<>* node);

        MEMBER_PTR(QStandardItemModel*, model);
    };

} // namespace gui

#endif // #ifndef GUI_WIDGET_OPT_EXPLORER_H
