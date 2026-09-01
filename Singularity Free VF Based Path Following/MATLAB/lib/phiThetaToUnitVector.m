function uVector = phiThetaToUnitVector(phi, theta)
    uVector = [
        cos(phi) * cos(theta);
        sin(phi) * cos(theta);
        sin(theta)
    ];
end 