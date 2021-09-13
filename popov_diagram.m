function popov_diagram(sys)
    % Plots de G(jw) transfer function of the system and the modified one 
    % for Popov stability analysis G_m(jw) = Re(G(jw)) + j*w*Im(G(jw))
    [re,im,w] = nyquist(sys);
    img = nyquistplot(sys);
    setoptions(img, 'ShowFullContour', 'off');
    re_gm = re(1,:);
    im_gm = w.*squeeze(im);
    hold on
    plot(re_gm, im_gm, 'red');
    legend('{G(j\omega)}', '{G^m(j\omega)}');
    title('Popov diagram')
    grid;
    hold off