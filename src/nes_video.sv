// NES video and sound to HDMI for the Tang Nano 20K. Adapted from fpga_game_consoles
// (platform/video/nes_video.sv); replaces nes2hdmi.sv on this board.
//
// Feeds the PPU's 256x240 picture into video_out.v (line ring buffer, scaler, menu, HDMI)
// and keeps the NES locked to the HDMI frame rate: the NES is paused once per frame, at
// the start of line 1, until HDMI is about to show line 0 (frame_go). The NES runs at
// about 60.1 Hz on its own and HDMI at 60.00 Hz, so each pause is a fraction of a line.

module nes_video (
    input             clk,              // NES clock
    input             resetn,
    input             running,          // the NES is out of reset

    // PPU and APU
    input      [5:0]  color,            // palette index of the current pixel
    input      [8:0]  cycle,
    input      [8:0]  scanline,
    input      [15:0] sample,
    output reg        pause = 0,        // hold the NES (frame lock)

    // Settings and menu (iosys)
    input      [1:0]  shape,
    input             overlay,
    output     [7:0]  overlay_x,
    output     [7:0]  overlay_y,
    input      [14:0] overlay_color,    // BGR5

    // HDMI
    input             hclk,
    input             hclk5,
    output            tmds_clk_n,
    output            tmds_clk_p,
    output     [2:0]  tmds_d_n,
    output     [2:0]  tmds_d_p
);

localparam AUDIO_RATE = 48000;
localparam [8:0] PAUSE_LINE = 9'd1;     // pause here: line 0 is complete
localparam [8:0] REARM_LINE = 9'd200;

// Pixels: one per PPU cycle on visible lines
reg [8:0] r_scanline, r_cycle;
reg       pix_we;
reg [7:0] pix_y, pix_x;
reg [5:0] pix_data;
always @(posedge clk) begin
    r_scanline <= scanline;
    r_cycle <= cycle;
    pix_we <= 1'b0;
    if ((r_scanline != scanline || r_cycle != cycle) && scanline < 9'd240 && ~cycle[8]) begin
        pix_we <= 1'b1;
        pix_y <= scanline[7:0];
        pix_x <= cycle[7:0];
        pix_data <= color;
    end
end

// Frame lock. While the menu is shown the pause is not released, so the game holds at
// line 1 of its next frame until the menu closes.
wire frame_go;
reg  locked = 0;                        // paused once this frame
always @(posedge clk) begin
    if (~resetn || ~running) begin
        pause <= 1'b0;
        locked <= 1'b0;
    end else begin
        if (~locked && ~pause && scanline == PAUSE_LINE)
            pause <= 1'b1;
        if (pause && frame_go && ~overlay) begin
            pause <= 1'b0;
            locked <= 1'b1;
        end
        if (scanline == REARM_LINE)
            locked <= 1'b0;
    end
end

// Audio: 48 kHz sample clock in hclk; nes_audio_hdmi brings the sample into hclk whole and
// removes its DC offset
localparam AUDIO_CLK_DELAY = 74250 * 1000 / AUDIO_RATE / 2;
reg [$clog2(AUDIO_CLK_DELAY)-1:0] audio_divider = 0;
reg clk_audio = 0;
always @(posedge hclk) begin
    if (audio_divider != AUDIO_CLK_DELAY - 1)
        audio_divider <= audio_divider + 1'b1;
    else begin
        clk_audio <= ~clk_audio;
        audio_divider <= 0;
    end
end

wire [15:0] audio;                      // signed, silence = 0
nes_audio_hdmi nes_audio (
    .clk(clk), .sample(sample),
    .clk_pixel(hclk), .clk_audio(clk_audio), .audio(audio)
);

// With the frame buffer the paused game is shown under the menu; without it, the 16-line
// ring buffer has no full picture, so there is no game under the menu.
`ifdef VIDEO_FRAMEBUFFER
localparam FRAMEBUFFER = 1;
`else
localparam FRAMEBUFFER = 0;
`endif

video_out #(.PIX_FMT(1), .DW(6), .AUDIO_RATE(AUDIO_RATE), .FRAMEBUFFER(FRAMEBUFFER)) video (
    .clk(clk), .resetn(resetn),
    .pix_we(pix_we), .pix_y(pix_y), .pix_x(pix_x), .pix_data(pix_data),
    .src_lines(9'd240), .src_active(running & (FRAMEBUFFER || ~overlay)), .shape(shape), .frame_go(frame_go),
    .overlay(overlay),
    .hclk(hclk), .hclk5(hclk5),
    .clk_audio(clk_audio), .audio_l(audio), .audio_r(audio),
    .overlay_x(overlay_x), .overlay_y(overlay_y), .overlay_color(overlay_color),
    .tmds_clk_n(tmds_clk_n), .tmds_clk_p(tmds_clk_p), .tmds_d_n(tmds_d_n), .tmds_d_p(tmds_d_p)
);

endmodule
