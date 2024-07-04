function d = disT_3d(x1, y1,z1, x2, y2,z2)

    x_d = (x1-x2).^2;
    y_d = (y1-y2).^2;
    z_d = (z1-z2).^2;
    d = sqrt(x_d + y_d + z_d);

end