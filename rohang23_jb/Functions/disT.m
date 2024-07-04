function d = disT(x1, y1, x2, y2)

    x_d = (x1-x2).^2;
    y_d = (y1-y2).^2;
    d = sqrt(x_d + y_d);

end