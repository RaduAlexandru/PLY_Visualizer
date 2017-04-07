/****************************************************************************
** Meta object code from reading C++ file 'Visualizer.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/Visualizer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Visualizer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Visualizer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      36,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x0a,
      23,   11,   11,   11, 0x0a,
      51,   11,   11,   11, 0x0a,
      76,   11,   11,   11, 0x0a,
     107,  102,   11,   11, 0x0a,
     153,   11,   11,   11, 0x0a,
     186,   11,   11,   11, 0x0a,
     219,   11,   11,   11, 0x0a,
     245,   11,   11,   11, 0x0a,
     283,   11,   11,   11, 0x0a,
     319,   11,   11,   11, 0x0a,
     350,   11,   11,   11, 0x0a,
     384,   11,   11,   11, 0x0a,
     415,   11,   11,   11, 0x0a,
     452,   11,   11,   11, 0x0a,
     484,   11,   11,   11, 0x0a,
     525,   11,   11,   11, 0x0a,
     561,  102,   11,   11, 0x0a,
     598,   11,   11,   11, 0x0a,
     631,   11,   11,   11, 0x0a,
     662,   11,   11,   11, 0x0a,
     697,  102,   11,   11, 0x0a,
     730,   11,   11,   11, 0x0a,
     765,   11,   11,   11, 0x0a,
     806,   11,   11,   11, 0x0a,
     845,   11,   11,   11, 0x0a,
     878,  102,   11,   11, 0x0a,
     917,  102,   11,   11, 0x0a,
     962,  102,   11,   11, 0x0a,
    1005,  102,   11,   11, 0x0a,
    1042,  102,   11,   11, 0x0a,
    1082,  102,   11,   11, 0x0a,
    1122,   11,   11,   11, 0x0a,
    1164,   11,   11,   11, 0x0a,
    1205,   11,   11,   11, 0x0a,
    1239,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Visualizer[] = {
    "Visualizer\0\0slotExit()\0"
    "on_loadFileButton_clicked()\0"
    "on_clearButton_clicked()\0"
    "on_unwrapButton_clicked()\0text\0"
    "on_colorComboBox_currentIndexChanged(QString)\0"
    "on_flatShadingCheckBox_clicked()\0"
    "on_perspectiveCheckBox_clicked()\0"
    "on_selectButton_clicked()\0"
    "on_showGridInactiveCheckBox_clicked()\0"
    "on_showGridActiveCheckBox_clicked()\0"
    "on_renderToImgButton_clicked()\0"
    "on_renderGridCellButton_clicked()\0"
    "on_renderWallsButton_clicked()\0"
    "on_renderGridWrappedButton_clicked()\0"
    "on_renderToFileButton_clicked()\0"
    "on_experimentalLoadingcheckBox_clicked()\0"
    "on_fixOrientationcheckBox_clicked()\0"
    "on_numWallsText_textChanged(QString)\0"
    "on_deformWallscheckBox_clicked()\0"
    "on_clearUnwrapButton_clicked()\0"
    "on_recomputeColorsButton_clicked()\0"
    "on_pathText_textChanged(QString)\0"
    "on_renderFullImgcheckBox_clicked()\0"
    "on_renderGridUnwrappedcheckBox_clicked()\0"
    "on_renderGridWrappedcheckBox_clicked()\0"
    "on_renderWallscheckBox_clicked()\0"
    "on_fullImgMagText_textChanged(QString)\0"
    "on_gridUnwrappedMagText_textChanged(QString)\0"
    "on_gridWrappedMagText_textChanged(QString)\0"
    "on_wallsMagText_textChanged(QString)\0"
    "on_aboveThreshText_textChanged(QString)\0"
    "on_belowThreshText_textChanged(QString)\0"
    "on_highCapDepthColorSlider_valueChanged()\0"
    "on_lowCapDepthColorSlider_valueChanged()\0"
    "on_loadFileConfigButton_clicked()\0"
    "grid_changed_slot()\0"
};

void Visualizer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Visualizer *_t = static_cast<Visualizer *>(_o);
        switch (_id) {
        case 0: _t->slotExit(); break;
        case 1: _t->on_loadFileButton_clicked(); break;
        case 2: _t->on_clearButton_clicked(); break;
        case 3: _t->on_unwrapButton_clicked(); break;
        case 4: _t->on_colorComboBox_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->on_flatShadingCheckBox_clicked(); break;
        case 6: _t->on_perspectiveCheckBox_clicked(); break;
        case 7: _t->on_selectButton_clicked(); break;
        case 8: _t->on_showGridInactiveCheckBox_clicked(); break;
        case 9: _t->on_showGridActiveCheckBox_clicked(); break;
        case 10: _t->on_renderToImgButton_clicked(); break;
        case 11: _t->on_renderGridCellButton_clicked(); break;
        case 12: _t->on_renderWallsButton_clicked(); break;
        case 13: _t->on_renderGridWrappedButton_clicked(); break;
        case 14: _t->on_renderToFileButton_clicked(); break;
        case 15: _t->on_experimentalLoadingcheckBox_clicked(); break;
        case 16: _t->on_fixOrientationcheckBox_clicked(); break;
        case 17: _t->on_numWallsText_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 18: _t->on_deformWallscheckBox_clicked(); break;
        case 19: _t->on_clearUnwrapButton_clicked(); break;
        case 20: _t->on_recomputeColorsButton_clicked(); break;
        case 21: _t->on_pathText_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 22: _t->on_renderFullImgcheckBox_clicked(); break;
        case 23: _t->on_renderGridUnwrappedcheckBox_clicked(); break;
        case 24: _t->on_renderGridWrappedcheckBox_clicked(); break;
        case 25: _t->on_renderWallscheckBox_clicked(); break;
        case 26: _t->on_fullImgMagText_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 27: _t->on_gridUnwrappedMagText_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 28: _t->on_gridWrappedMagText_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 29: _t->on_wallsMagText_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 30: _t->on_aboveThreshText_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 31: _t->on_belowThreshText_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 32: _t->on_highCapDepthColorSlider_valueChanged(); break;
        case 33: _t->on_lowCapDepthColorSlider_valueChanged(); break;
        case 34: _t->on_loadFileConfigButton_clicked(); break;
        case 35: _t->grid_changed_slot(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Visualizer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Visualizer::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_Visualizer,
      qt_meta_data_Visualizer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Visualizer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Visualizer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Visualizer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Visualizer))
        return static_cast<void*>(const_cast< Visualizer*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int Visualizer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 36)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 36;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
