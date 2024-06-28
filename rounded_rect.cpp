#include "rounded_rect.h"

Rounded_Rect::Rounded_Rect(QWidget *parent) : QWidget(parent) {}

void Rounded_Rect::paintEvent(QPaintEvent *) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);  // Optional: Enable antialiasing for smoother edges
    painter.setBrush(QColor(60,60,60,200));  // Set background color
    painter.setPen(Qt::NoPen);    // No border

    // Define the rectangle to be painted
    //QRectF rect = QRectF(0.0, 0.0, QWidget::width(), QWidget::height());

    // Draw a rounded rectangle that matches the widget's dimensions
    //int radius = 10;  // Adjust the radius as needed
    //painter.drawRoundedRect(rect, radius, radius);
}
