function [output] = norm_pdf(x,mu,sigma)
    output = 1/(sigma*sqrt(2*pi)) * exp(-(x - mu).^2 / (2*sigma^2));
    output = output/sum(output);
end

