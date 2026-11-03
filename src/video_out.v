// Video output for the Tang Nano 20K: line ring buffer, scaler, menu overlay and HDMI
// (1280x720 at 60 Hz). Adapted from fpga_game_consoles (platform/video/video_out.v); the menu
// is tangcore's textdisp (in iosys_bl616) instead of osd.v.
//
// The core writes each visible line into a 16-line ring buffer in its own clock domain.
// The HDMI side reads it back at 74.25 MHz:
// - Vertical: exactly 3x. 224, 239 or 240 source lines, centered (672, 717 or 720 lines).
// - Horizontal: 256 source pixels become W output pixels. An output pixel that straddles
//   two source pixels is a mix of both, weighted by how much of it each one covers, so
//   every source pixel keeps the same width. W = 878 for 8:7 pixels (what the consoles
//   produced), 768 for square pixels, 4 * lines for 4:3.
// - Menu: textdisp's 32x28 characters (256x224 pixels) at 3x in a fixed 768x672 window,
//   whatever the game's scaling. While the menu is shown, the game is dimmed to 1/4 and
//   shows through the menu's black pixels.
// - Output: 0-255 RGB, or limited range (16-235) with RGB_LIMITED; the AVI InfoFrame must
//   then declare it.
//
// The core runs slightly faster than HDMI. Once a frame the core pauses after writing line 0
// and waits for frame_go, which comes when HDMI is about to show source line 0. The core then
// stays ahead of the reader by 1 to about 15 lines, which the 16-line ring holds. See
// nes_video.sv.

module video_out #(
    parameter PIX_FMT = 0,              // pixel format: 0 = BGR555 (SNES), 1 = NES palette index
    parameter DW = 15,                  // line buffer pixel width
    parameter AUDIO_RATE = 48000,
    parameter RGB_LIMITED = 0           // 1: send limited range (16-235)
) (
    // Core clock domain
    input             clk,
    input             resetn,
    input             pix_we,           // write one pixel
    input      [7:0]  pix_y,            // source line; bits 3:0 pick the ring buffer slot
    input      [7:0]  pix_x,
    input  [DW-1:0]   pix_data,
    input      [8:0]  src_lines,        // visible source lines: 224, 239 or 240
    input             src_active,       // 0: no game picture yet, show black
    input      [1:0]  shape,            // pixel shape: 0 = 8:7, 1 = square, 2 = 4:3
    output reg        frame_go,         // one-cycle pulse: HDMI is about to show source line 0
    input             overlay,          // show the menu

    // HDMI clock domain
    input             hclk,             // 74.25 MHz pixel clock
    input             hclk5,            // 5x pixel clock for the serializers
    input             clk_audio,
    input      [15:0] audio_l,
    input      [15:0] audio_r,

    // Menu (textdisp): position in, BGR5 color out 2 cycles later
    output reg [7:0]  overlay_x,
    output reg [7:0]  overlay_y,
    input      [14:0] overlay_color,

    output            tmds_clk_n,
    output            tmds_clk_p,
    output     [2:0]  tmds_d_n,
    output     [2:0]  tmds_d_p
);

localparam AUDIO_BIT_WIDTH = 16;

// Pipeline depth from the position counter (stage 0) to the rgb register read by the HDMI
// encoder. Stage 0 works on pixel hpos = cx + LAT so that its result lands on column hpos.
localparam LAT = 7;

// Line state for the next line is computed here, in horizontal blanking.
localparam [10:0] LINE_EVT = 11'd1290;

wire [10:0] cx;
wire [9:0]  cy;

//
// Line ring buffer: 16 lines of 256 pixels
//
reg [DW-1:0] mem [0:4095];
always @(posedge clk)
    if (pix_we) mem[{pix_y[3:0], pix_x}] <= pix_data;

wire [11:0]  rd_addr;
reg [DW-1:0] q;
always @(posedge hclk)
    q <= mem[rd_addr];

//
// Settings from the core clock domain. They change rarely, so two flip-flops per bit are
// enough; the geometry is taken from them once per frame, in vertical blanking.
//
reg [12:0] cfg_s1, cfg_s2;
always @(posedge hclk)
    {cfg_s2, cfg_s1} <= {cfg_s1, {overlay, src_active, src_lines, shape}};
wire       cfg_overlay = cfg_s2[12];
wire       cfg_active  = cfg_s2[11];
wire [8:0] cfg_lines   = cfg_s2[10:2];
wire [1:0] cfg_shape   = cfg_s2[1:0];

// Picture geometry (power-up values: 224 lines, 8:7)
reg [8:0]  geo_v    = 9'd224;           // source lines
reg [9:0]  geo_y0   = 10'd24;           // first output line of the picture
reg [10:0] geo_w    = 11'd878;          // picture width in output pixels
reg [10:0] geo_xpre = 11'd200;          // geo_x0 - 1: fetch source pixel 0
reg [10:0] geo_x0   = 11'd201;          // first output column of the picture
reg [10:0] geo_xend = 11'd1079;         // geo_x0 + geo_w

wire [8:0] v_clamped = cfg_lines > 9'd240 ? 9'd240 : cfg_lines < 9'd16 ? 9'd16 : cfg_lines;
wire [9:0] v_x3 = {1'b0, v_clamped} + {v_clamped, 1'b0};

always @(posedge hclk) begin
    if (cx == 0 && cy == 10'd720) begin
        reg [10:0] w, x0;
        case (cfg_shape)
        2'd1:    begin w = 11'd768;               x0 = 11'd256; end
        2'd2:    begin w = {v_clamped, 2'b0};      x0 = 11'd640 - {v_clamped, 1'b0}; end
        default: begin w = 11'd878;               x0 = 11'd201; end
        endcase
        geo_v    <= v_clamped;
        geo_y0   <= (10'd720 - v_x3) >> 1;
        geo_w    <= w;
        geo_x0   <= x0;
        geo_xpre <= x0 - 11'd1;
        geo_xend <= x0 + w;
    end
end

//
// Vertical: which source line each output line shows
//
wire [9:0] next_line = cy == 10'd749 ? 10'd0 : cy + 10'd1;
reg        line_on = 0;                 // the output line shows the picture
reg [7:0]  sy = 0;                      // source line
reg [1:0]  sy_ph = 0;                   // 0..2: which of the 3 output lines of sy
reg        go_t = 0;                    // toggles at each frame_go

always @(posedge hclk) begin
    if (cx == LINE_EVT) begin
        if (next_line == geo_y0) begin
            line_on <= 1'b1;
            sy <= 0;
            sy_ph <= 0;
            go_t <= ~go_t;
        end else if (line_on) begin
            if (sy_ph == 2'd2) begin
                sy_ph <= 0;
                sy <= sy + 8'd1;
                if ({1'b0, sy} == geo_v - 9'd1)
                    line_on <= 1'b0;
            end else
                sy_ph <= sy_ph + 2'd1;
        end
    end
end

reg [2:0] go_s = 0;
always @(posedge clk) begin
    go_s <= {go_s[1:0], go_t};
    frame_go <= go_s[2] ^ go_s[1];
end

//
// Horizontal: stage 0 steps through the source pixels.
// Units: one source pixel is geo_w long and one output pixel is 256 long, so 256 source
// pixels span exactly geo_w output pixels. acc is where the current output pixel starts
// inside source pixel A (= bidx - 1). If it ends past A, the part past A (t) lies in
// source pixel B (= bidx) and becomes B's weight.
//
wire [10:0] hpos = cx + LAT;            // output column this pipeline cycle produces
wire pre = line_on && hpos == geo_xpre; // fetch source pixel 0, before the first column
wire win = line_on && hpos >= geo_x0 && hpos < geo_xend;

reg [10:0] acc;
reg [8:0]  bidx;                        // index of source pixel B
reg        take_n;                      // the next output pixel moves to the next A
wire signed [12:0] t = $signed({2'b0, acc}) + 13'sd256 - $signed({2'b0, geo_w});

assign rd_addr = {sy[3:0], pre ? 8'd0 : bidx[7:0]};

reg       s1_on, s1_take;
reg [7:0] s1_w;

always @(posedge hclk) begin
    s1_on <= 0;
    s1_take <= 0;
    s1_w <= 0;
    if (pre) begin
        acc <= 0;
        bidx <= 9'd1;
        take_n <= 1'b1;
    end else if (win) begin
        s1_on <= 1'b1;
        s1_take <= take_n;
        s1_w <= t > 0 ? t[7:0] : 8'd0;
        if (t >= 0) begin
            acc <= t[10:0];
            bidx <= bidx + 9'd1;
            take_n <= 1'b1;
        end else begin
            acc <= acc + 11'd256;
            take_n <= 1'b0;
        end
    end
end

//
// Stage 1: q holds the source pixel read in stage 0. Decode it to RGB888.
//
reg [23:0] q_rgb;
generate
if (PIX_FMT == 1) begin : g_nes
    always @* q_rgb = nes_palette(q[5:0]);
end else begin : g_snes
    // BGR555 to RGB888, low bits repeated so that 31 becomes 255
    always @* q_rgb = {q[4:0], q[4:2], q[9:5], q[9:7], q[14:10], q[14:12]};
end
endgenerate

// Stage 2: source pixels A and B of this output pixel
reg [23:0] a2, b2;
reg [7:0]  w2;
reg        on2;
always @(posedge hclk) begin
    b2 <= q_rgb;
    if (s1_take) a2 <= b2;
    w2 <= s1_w;
    on2 <= s1_on;
end

// Stage 3: B - A per channel
reg signed [8:0] d3_r, d3_g, d3_b;
reg [23:0] a3;
reg [7:0]  w3;
reg        on3;
always @(posedge hclk) begin
    d3_r <= $signed({1'b0, b2[23:16]}) - $signed({1'b0, a2[23:16]});
    d3_g <= $signed({1'b0, b2[15:8]})  - $signed({1'b0, a2[15:8]});
    d3_b <= $signed({1'b0, b2[7:0]})   - $signed({1'b0, a2[7:0]});
    a3 <= a2;
    w3 <= w2;
    on3 <= on2;
end

// Stage 4: (B - A) * weight
reg signed [17:0] p4_r, p4_g, p4_b;
reg [23:0] a4;
reg        on4;
wire signed [9:0] w3s = {2'b0, w3};
always @(posedge hclk) begin
    p4_r <= d3_r * w3s;
    p4_g <= d3_g * w3s;
    p4_b <= d3_b * w3s;
    a4 <= a3;
    on4 <= on3;
end

// Stage 5: A + (B - A) * weight / 256, rounded. The result stays between A and B.
function [7:0] mix(input [7:0] a, input signed [17:0] p);
    reg signed [17:0] s;
    begin
        s = $signed({10'b0, a}) + ((p + 18'sd128) >>> 8);
        mix = s[7:0];
    end
endfunction

reg [23:0] g5;
reg        on5;
always @(posedge hclk) begin
    g5 <= {mix(a4[23:16], p4_r), mix(a4[15:8], p4_g), mix(a4[7:0], p4_b)};
    on5 <= on4;
end

//
// Menu window, aligned to stage 5. textdisp returns the color of position (overlay_x,
// overlay_y) 2 cycles later and needs at least 3 cycles per x step. Stage 5 is on column
// hpos - 5, so overlay_x is (hpos - MENU_X0 - 3) / 3: its color then arrives for the column
// stage 5 is on. overlay_x goes 255 -> 0 at the window edge so that textdisp sees x change.
//
localparam [10:0] MENU_X0 = 11'd256;    // window left edge
localparam [9:0]  MENU_Y0 = 10'd24;     // window top edge

reg [1:0] mx_ph, my_ph;
always @(posedge hclk) begin
    if (hpos == MENU_X0 - 11'd1) begin
        overlay_x <= 8'd255;
        mx_ph <= 0;
    end else if (mx_ph == 2'd2) begin
        overlay_x <= overlay_x + 8'd1;
        mx_ph <= 0;
    end else
        mx_ph <= mx_ph + 2'd1;

    if (cx == LINE_EVT) begin
        if (next_line == MENU_Y0) begin
            overlay_y <= 0;
            my_ph <= 0;
        end else if (my_ph == 2'd2) begin
            overlay_y <= overlay_y + 8'd1;
            my_ph <= 0;
        end else
            my_ph <= my_ph + 2'd1;
    end
end

wire menu_win0 = hpos >= MENU_X0 && hpos < MENU_X0 + 11'd768 &&
                 cy >= MENU_Y0 && cy < MENU_Y0 + 10'd672;
reg [4:0] menu_win_d;
always @(posedge hclk)
    menu_win_d <= {menu_win_d[3:0], menu_win0};
wire menu_win5 = menu_win_d[4];

// BGR5 to RGB888, low bits repeated so that 31 becomes 255
wire [23:0] menu_rgb = {overlay_color[4:0],   overlay_color[4:2],
                        overlay_color[9:5],   overlay_color[9:7],
                        overlay_color[14:10], overlay_color[14:12]};

// Stage 6: the picture on the 0-255 scale: game, menu, black around the picture.
reg [23:0] rgb_full;
always @(posedge hclk) begin
    reg [23:0] game;
    game = on5 && cfg_active ? g5 : 24'h000000;
    if (cfg_overlay)
        rgb_full <= menu_win5 && overlay_color != 15'd0 ? menu_rgb
                  : {2'b0, game[23:18], 2'b0, game[15:10], 2'b0, game[7:2]};
    else
        rgb_full <= game;
end

// Stage 7: what the HDMI encoder sends. With RGB_LIMITED, 0 becomes 16 (black) and 255
// becomes 235 (white). x * 220 / 256 is within 1 of CTA-861's round(x * 219 / 255) for
// every x, exact at both ends, and needs only two subtractions: x * 220 = x * 256 - x * 32
// - x * 4.
function [7:0] to_limited(input [7:0] x);
    reg [15:0] p;
    begin
        p = {x, 8'd0} - {3'd0, x, 5'd0} - {6'd0, x, 2'd0} + 16'd128;
        to_limited = 8'd16 + p[15:8];
    end
endfunction

function [23:0] limited_rgb(input [23:0] c);
    limited_rgb = {to_limited(c[23:16]), to_limited(c[15:8]), to_limited(c[7:0])};
endfunction

reg [23:0] rgb;
always @(posedge hclk)
    rgb <= RGB_LIMITED ? limited_rgb(rgb_full) : rgb_full;

//
// HDMI
//
wire [9:0]  frame_height;
wire [10:0] frame_width;
wire [2:0]  tmds;
wire        tmds_clock;
reg [15:0]  audio_sample_word [1:0];
always @(posedge hclk) begin
    audio_sample_word[0] <= audio_l;
    audio_sample_word[1] <= audio_r;
end

hdmi #( .VIDEO_ID_CODE(4),
        .DVI_OUTPUT(0),
        .VIDEO_REFRESH_RATE(60.0),
        .IT_CONTENT(1),
        .AUDIO_RATE(AUDIO_RATE),
        .AUDIO_BIT_WIDTH(AUDIO_BIT_WIDTH),
        .START_X(0),
        .START_Y(0) )
hdmi (  .clk_pixel_x5(hclk5),
        .clk_pixel(hclk),
        .clk_audio(clk_audio),
        .rgb(rgb),
        // Straight from the core clock domain, as in nes2hdmi/snes2hdmi. It also resets the
        // serializers, and through a flip-flop on hclk Gowin times that release against
        // hclk5 and reports a recovery failure.
        .reset(~resetn),
        .audio_sample_word(audio_sample_word),
        .tmds(tmds),
        .tmds_clock(tmds_clock),
        .cx(cx),
        .cy(cy),
        .frame_width(frame_width),
        .frame_height(frame_height) );

// Gowin LVDS output buffer
ELVDS_OBUF tmds_bufds [3:0] (
    .I({hclk, tmds}),
    .O({tmds_clk_p, tmds_d_p}),
    .OB({tmds_clk_n, tmds_d_n})
);

// 2C02 palette: https://www.nesdev.org/wiki/PPU_palettes
function [23:0] nes_palette(input [5:0] i);
    case (i)
    6'd0:  nes_palette = 24'h545454;  6'd1:  nes_palette = 24'h001e74;  6'd2:  nes_palette = 24'h081090;  6'd3:  nes_palette = 24'h300088;
    6'd4:  nes_palette = 24'h440064;  6'd5:  nes_palette = 24'h5c0030;  6'd6:  nes_palette = 24'h540400;  6'd7:  nes_palette = 24'h3c1800;
    6'd8:  nes_palette = 24'h202a00;  6'd9:  nes_palette = 24'h083a00;  6'd10: nes_palette = 24'h004000;  6'd11: nes_palette = 24'h003c00;
    6'd12: nes_palette = 24'h00323c;  6'd13: nes_palette = 24'h000000;  6'd14: nes_palette = 24'h000000;  6'd15: nes_palette = 24'h000000;
    6'd16: nes_palette = 24'h989698;  6'd17: nes_palette = 24'h084cc4;  6'd18: nes_palette = 24'h3032ec;  6'd19: nes_palette = 24'h5c1ee4;
    6'd20: nes_palette = 24'h8814b0;  6'd21: nes_palette = 24'ha01464;  6'd22: nes_palette = 24'h982220;  6'd23: nes_palette = 24'h783c00;
    6'd24: nes_palette = 24'h545a00;  6'd25: nes_palette = 24'h287200;  6'd26: nes_palette = 24'h087c00;  6'd27: nes_palette = 24'h007628;
    6'd28: nes_palette = 24'h006678;  6'd29: nes_palette = 24'h000000;  6'd30: nes_palette = 24'h000000;  6'd31: nes_palette = 24'h000000;
    6'd32: nes_palette = 24'heceeec;  6'd33: nes_palette = 24'h4c9aec;  6'd34: nes_palette = 24'h787cec;  6'd35: nes_palette = 24'hb062ec;
    6'd36: nes_palette = 24'he454ec;  6'd37: nes_palette = 24'hec58b4;  6'd38: nes_palette = 24'hec6a64;  6'd39: nes_palette = 24'hd48820;
    6'd40: nes_palette = 24'ha0aa00;  6'd41: nes_palette = 24'h74c400;  6'd42: nes_palette = 24'h4cd020;  6'd43: nes_palette = 24'h38cc6c;
    6'd44: nes_palette = 24'h38b4cc;  6'd45: nes_palette = 24'h3c3c3c;  6'd46: nes_palette = 24'h000000;  6'd47: nes_palette = 24'h000000;
    6'd48: nes_palette = 24'heceeec;  6'd49: nes_palette = 24'ha8ccec;  6'd50: nes_palette = 24'hbcbcec;  6'd51: nes_palette = 24'hd4b2ec;
    6'd52: nes_palette = 24'hecaeec;  6'd53: nes_palette = 24'hecaed4;  6'd54: nes_palette = 24'hecb4b0;  6'd55: nes_palette = 24'he4c490;
    6'd56: nes_palette = 24'hccd278;  6'd57: nes_palette = 24'hb4de78;  6'd58: nes_palette = 24'ha8e290;  6'd59: nes_palette = 24'h98e2b4;
    6'd60: nes_palette = 24'ha0d6e4;  6'd61: nes_palette = 24'ha0a2a0;  6'd62: nes_palette = 24'h000000;  default: nes_palette = 24'h000000;
    endcase
endfunction

endmodule
