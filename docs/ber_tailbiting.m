function ber_tailbiting()
% Opulent Voice: the tail-bits story in three curves.
% Domain model: a convolutional CODE has ONE intrinsic BER curve -- the union
% bound (from its distance spectrum). A FRAME is a boundary condition on it:
%   unterminated -> trellis just stops; last bits decode with no future
%                   context -> high-SNR error FLOOR (the tail).
%   tail-biting  -> seed start state = last 6 payload bits so start==end; the
%                   trellis closes into a RING. No exposed end -> no floor,
%                   at ZERO extra bits (fits the locked 40 ms frame).
% Verified in Python: last-16-bit error share 72% -> 2%, full-frame BER ~3.5x.

    trellis = poly2trellis(7,[171 133]);   % TRUE Voyager, d_free = 10
    K=1072; M=6; tblen=34; nsdec=3; W=48; puncpat=[1;1];

    % (1) Union bound: the code's promise (info-weight spectrum of 171/133)
    d_spec=[10 12 14 16]; c_spec=[36 211 1404 11633];
    eb=0:0.1:7;
    bound=arrayfun(@(g) min(sum(c_spec.*qfunc(sqrt(d_spec*10^(g/10)))),0.5), eb);

    % (2) Monte Carlo: full-frame BER, unterminated vs tail-biting
    EbN0=0:0.5:6; ber_un=nan(size(EbN0)); ber_tb=nan(size(EbN0));
    for i=1:numel(EbN0)
        sigma=sqrt(1/(2*10^((EbN0(i)+10*log10(1/2))/10)));
        eU=0; eT=0; nb=0; f=0;
        while (eU<200 || eT<80) && f<3000
            u=randi([0 1],K,1);

            % unterminated: zero init, no tail; count ALL K bits
            d0=vitdec(softq(convenc(u,trellis),sigma,nsdec),trellis,tblen,'trunc','soft',nsdec);
            eU=eU+sum(d0~=u);

            % tail-biting: start state = last M bits (ring closes)
            [~,s0]=convenc(u(end-M+1:end),trellis);
            qtb=softq(convenc(u,trellis,puncpat,s0),sigma,nsdec);   % length 2K
            % wrap-around Viterbi: warmup W info-bits (2W soft) on BOTH sides
            qw=[qtb(end-2*W+1:end); qtb; qtb(1:2*W)];
            dw=vitdec(qw,trellis,tblen,'trunc','soft',nsdec);
            dtb=dw(W+1:W+K);                                        % middle = answer
            eT=eT+sum(dtb~=u);

            nb=nb+K; f=f+1;
        end
        if eU>0, ber_un(i)=eU/nb; end
        if eT>0, ber_tb(i)=eT/nb; end
        fprintf('Eb/N0 %.1f dB : unterminated %.2e   tail-biting %.2e\n',EbN0(i),eU/nb,eT/nb);
    end

    % (3) The three-curve story
    figure; semilogy(eb,qfunc(sqrt(2*10.^(eb/10))),'k--','LineWidth',1.2); hold on; grid on;
    semilogy(eb,bound,'Color',[0 .5 0],'LineWidth',2);
    semilogy(EbN0,ber_un,'rs-','LineWidth',1.5,'MarkerFaceColor','r');
    semilogy(EbN0,ber_tb,'go-','LineWidth',1.5,'MarkerFaceColor','g');
    ylim([1e-7 1]); xlim([0 7]); xlabel('E_b/N_0 (dB)'); ylabel('BER');
    legend('uncoded (theory)','union bound (code promise, d_{free}=10)',...
           'full frame, UNTERMINATED (tail floor)',...
           'full frame, TAIL-BITING (rides the bound)','Location','southwest');
    title('Opulent Voice: the tail floor and its cure');
end

function q = softq(code, sigma, nsdec)   % your radio's quantizer (opv_demod.hpp)
    x=1-2*double(code); y=x+sigma*randn(size(x));
    scale=mean(abs(y)); n=(-y./scale)*3.5+3.5;
    q=min(max(floor(n+0.5),0),2^nsdec-1);
end