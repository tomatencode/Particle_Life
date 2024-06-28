#include <QWidget>
#include <QPainter>

class Rounded_Rect : public QWidget {
public:
    Rounded_Rect(QWidget *parent = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;
};
