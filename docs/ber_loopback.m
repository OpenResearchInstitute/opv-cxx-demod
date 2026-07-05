function ber_loopback()
    trellis = poly2trellis(7, [171 133]);   % TRUE Voyager, d_free = 10
    K       = 1072;  EbN0 = 0:0.5:6;  tblen = 34;  nsdec = 3;
    ber_soft = zeros(size(EbN0));  ber_hard = zeros(size(EbN0));

    for i = 1:numel(EbN0)
        EcN0  = EbN0(i) + 10*log10(1/2);      % rate-1/2: noise at coded-bit energy
        sigma = sqrt(1/(2*10^(EcN0/10)));
        nErrS = 0; nErrH = 0; nBits = 0;

        while nErrS < 300 && nBits < 5e6
            u = randi([0 1], K, 1);
            c = convenc(u, trellis);
            x = 1 - 2*c;                       % BPSK: 0->+1, 1->-1
            y = x + sigma*randn(size(x));      % AWGN

            % ---- YOUR quantizer, ported exactly from opv_demod.hpp ----
            scale = mean(abs(y));              % per-frame mean-|soft| AGC
            n     = (-y ./ scale) * 3.5 + 3.5; % same map as the C++
            qsym  = min(max(floor(n + 0.5), 0), 7);   % 0..7, round-half-up + clamp
            dS = vitdec(qsym, trellis, tblen, 'trunc', 'soft', nsdec);

            % ---- hard-decision anchor (polarity-unambiguous) ----
            dH = vitdec(double(y < 0), trellis, tblen, 'trunc', 'hard');

            nErrS = nErrS + sum(dS(1:K-6) ~= u(1:K-6));
            nErrH = nErrH + sum(dH(1:K-6) ~= u(1:K-6));
            nBits = nBits + (K-6);
        end
        ber_soft(i) = nErrS/max(nBits,1);  ber_hard(i) = nErrH/max(nBits,1);
        fprintf('Eb/N0 %.1f dB : soft %.2e   hard %.2e\n', EbN0(i), ber_soft(i), ber_hard(i));
    end

    ebn0_lin = 10.^(EbN0/10);
    figure; semilogy(EbN0, qfunc(sqrt(2*ebn0_lin)), 'k--', ...
                     EbN0, ber_hard, 'bs-', EbN0, ber_soft, 'go-', 'LineWidth', 1.5);
    grid on; xlabel('E_b/N_0 (dB)'); ylabel('BER'); ylim([1e-6 1]);
    legend('uncoded (theory)','coded 171/133 (hard)','coded 171/133 (soft, your quantizer)');
    title('Opulent Voice loopback BER — true Voyager 171/133');
end