/* ═══════════════════════════════════════════════════════════════════
 *  cpcu_tui_render.c + cpcu_io.c — BATTERY REMOVAL PATCHES
 *
 *  Battery is no longer sampled by BSAU. Remove display to stop
 *  flickering. wireless_packet.h is NOT touched — the vbat_raw
 *  field stays in the packet struct, just ignored.
 * ═══════════════════════════════════════════════════════════════════ */


/* ── PATCH 1: cpcu_tui_render.c — Health banner (line ~669) ──
 *
 * REPLACE these lines:
 *   int batt  = (latest.vbat_raw > 0 && batt_v < 2.7f) ? 2
 *             : (latest.vbat_raw > 0 && batt_v < 3.0f) ? 1 : 0;
 *
 * WITH: */
        int batt  = 0;  /* battery monitoring disabled (BSAU not sampling) */


/* ── PATCH 2: cpcu_tui_render.c — Health banner pills (line ~687) ──
 *
 * REMOVE this line:
 *   PILL("batt",   batt);
 *
 * And remove 'batt' from the stats array (line ~693):
 *
 * REPLACE:
 *   int stats[] = { radio, ioh, ipcs, batt, dsph, fsm };
 * WITH: */
        int stats[] = { radio, ioh, ipcs, dsph, fsm };


/* ── PATCH 3: cpcu_tui_render.c — Battery section on page 1 (lines 744-753) ──
 *
 * REPLACE:
 *   draw_section(r, g_col_r, "BATTERY (BSAU pack)");
 *   ...
 *   draw_lv(r,     g_col_r, "Voltage:", ...
 *   draw_lv(r + 1, g_col_r, "Raw ADC:", ...
 *   draw_lv(r + 2, g_col_r, "Level:",   ...
 *
 * WITH: */
    draw_section(r, g_col_r, "SYSTEM");
    /* (battery section removed — BSAU no longer samples vbat) */
    /* reuse this space for ring/overflow stats that were below */


/* ── PATCH 4: cpcu_tui_render.c — Remove batt_v variable (line 646) ──
 *
 * REMOVE this line (it reads the unused field):
 *   float batt_v = latest.vbat_raw * (3.3f / 4095.0f) * 2.0f;
 */


/* ── PATCH 5: cpcu_safety.c — Disable battery critical check ──
 *
 * In SAFETY_UpdateFromPacket() (line ~161-167), the battery check
 * is already force-cleared in cpcu_io.c (line 437-440):
 *
 *   safety.battery.critical = false;
 *
 * This is already in place. No change needed in cpcu_safety.c.
 * The force-clear in cpcu_io.c prevents stale/zero vbat_raw
 * from triggering SAFE state.
 */


/* ═══════════════════════════════════════════════════════════════════
 *  WEB DASHBOARD — Show URL + SSH tunnel command in tmux
 * ═══════════════════════════════════════════════════════════════════ */

/* ── PATCH 6: launch.sh — Web dashboard URL display ──
 *
 * In launch.sh, find ALL instances of:
 *
 *   log "Web dashboard at http://$(hostname -I | awk '{print $1}'):8765"
 *
 * (there are 3, around lines 707, 733, 769)
 *
 * REPLACE each with:
 */

/*
    local pi_ip=$(hostname -I | awk '{print $1}')
    log "═══════════════════════════════════════════════════════"
    log "Web dashboard running on port 8765"
    log ""
    log "  Same network:  http://${pi_ip}:8765"
    log ""
    log "  Remote (SSH):   On your PC run:"
    log "    ssh -L 8765:localhost:8765 $(whoami)@${pi_ip}"
    log "    Then open:  http://localhost:8765"
    log "═══════════════════════════════════════════════════════"
*/

/* ── PATCH 7: cpcu_ws.c — Print URL on startup (line ~582) ──
 *
 * The WS server already prints a bind message. After it, add: */

/*
    char ip[64] = {0};
    FILE *f = popen("hostname -I | awk '{print $1}'", "r");
    if(f) { fgets(ip, sizeof(ip), f); pclose(f); }
    /* trim newline */
    for(int i = 0; ip[i]; i++) if(ip[i] == '\n') ip[i] = '\0';

    fprintf(stderr,
        "\n"
        "  ══════════════════════════════════════════════\n"
        "  WEB DASHBOARD\n"
        "  Same network:  http://%s:8765\n"
        "  Remote (SSH):   ssh -L 8765:localhost:8765 %s@%s\n"
        "                  then http://localhost:8765\n"
        "  ══════════════════════════════════════════════\n\n",
        ip, getenv("USER") ? getenv("USER") : "pi", ip);
*/
