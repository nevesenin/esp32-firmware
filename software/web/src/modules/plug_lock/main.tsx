/* esp32-firmware
 * Copyright (C) 2026 empunkt <empunkt@mailbox.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

import * as util from "../../ts/util";
import * as API from "../../ts/api";
import { h, Fragment, Component } from "preact";
import { __ } from "../../ts/translation";
import { ConfigComponent } from "../../ts/components/config_component";
import { SubPage } from "../../ts/components/sub_page";
import { NavbarItem } from "../../ts/components/navbar_item";
import { FormRow } from "../../ts/components/form_row";
import { Switch } from "../../ts/components/switch";
import { Alert } from "react-bootstrap";
import { PlugLockActuation } from "./generated/plug_lock_actuation.enum";
import { PlugLockLoop } from "./generated/plug_lock_loop.enum";

// A page of its own rather than a row in EVSE Settings, and the reason is the Save
// button. Every other row on that page is collected in component state and written
// when Save is pressed; this setting was written on click, which is a second, silent
// contract inside one form. Here the one switch owns the form, so pressing Save means
// exactly one thing and the one-way-door warning can stay on screen permanently.

interface PlugLockNavbarState {
    config: API.getType["plug_lock/config"];
    state: API.getType["plug_lock/state"];
}

export class PlugLockNavbar extends Component<{}, PlugLockNavbarState> {
    constructor() {
        super();

        // NavbarItem does not subscribe to the API on its own, and both values feeding
        // the hidden rule can change while the interface is open - a Bricklet can be
        // unplugged at any time. Without these the entry would keep whatever
        // visibility it had when the app last rendered for some other reason.
        util.addApiEventListener('plug_lock/config', () => {
            this.setState({config: API.get('plug_lock/config')});
        });

        util.addApiEventListener('plug_lock/state', () => {
            this.setState({state: API.get('plug_lock/state')});
        });
    }

    render(props: {}, s: PlugLockNavbarState) {
        const found = s.state?.relay_found && s.state?.input_found;

        return <NavbarItem
            name="plug_lock"
            module="plug_lock"
            // Hidden, not disabled: without both Bricklets the plug lock cannot be
            // used at all, and a menu entry leading to a permanently refused switch
            // would only raise questions. Enabling is refused by the EVSE in any case.
            //
            // Unless it is already enabled. Then the Bricklets have gone missing from
            // a wallbox that is now refusing to charge, and this page is the only way
            // out - hiding it would leave the user with a blinking error and nothing
            // to act on.
            hidden={!found && !s.config?.enabled}
            title={__("plug_lock.navbar.plug_lock")}
            // The Wallbox group's Type 2 connector (evse_group/main.tsx) at full size
            // with a padlock badge, because Feather has no connector glyph and this
            // page is about that connector specifically.
            //
            // The badge overlaps the connector, so the connector is cut back around
            // it. The cut is a single circle concentric with the badge rather than the
            // badge's own outline offset outwards: cutting along the padlock silhouette
            // leaves the pin it touches as a compound sliver, while one arc leaves a
            // clean crescent that reads as deliberate. This is how MDI's eye-lock and
            // its siblings handle the same collision.
            //
            // A knockout in the background colour would avoid the cut entirely, but
            // that colour changes on hover and when the entry is active, so there is
            // no single value to knock out to.
            //
            // Geometry: badge bounding radius 4.17 about (20.1, 20.6), plus a 1.1 gap.
            symbol={
                <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="currentColor" stroke="none" class="feather feather-type2-lock">
                    <mask id="plug_lock_navbar_badge_cut">
                        <rect x="0" y="0" width="24" height="24" fill="#fff"/>
                        <circle cx="20.1" cy="20.6" r="5.27" fill="#000"/>
                    </mask>
                    <g mask="url(#plug_lock_navbar_badge_cut)">
                        <path d="M23 10.846c0 6.022-4.925 10.904-11 10.904S1 16.868 1 10.846c0-1.506.308-2.94.864-4.244C2.143 5.95 2.88 4.75 2.88 4.75h18.243s.736 1.2 1.014 1.852c.556 1.304.864 2.738.864 4.244z" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                        <circle cx="9" cy="8.5" r="1.5"/>
                        <circle cx="15" cy="8.5" r="1.5"/>
                        <circle cx="9" cy="16.75" r="2"/>
                        <circle cx="15" cy="16.75" r="2"/>
                        <circle cx="6" cy="12" r="2"/>
                        <circle cx="12" cy="12" r="2"/>
                        <circle cx="18" cy="12" r="2"/>
                    </g>
                    <path d="M18.55 19.4v-1.35a1.55 1.55 0 0 1 3.1 0v1.35" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round"/>
                    <rect x="17" y="19.4" width="6.2" height="4" rx="1"/>
                </svg>
            }
        />;
    }
}

type PlugLockConfig = API.getType["plug_lock/config"];

interface PlugLockPageState {
    plug_lock_state: API.getType["plug_lock/state"];
}

export class PlugLock extends ConfigComponent<'plug_lock/config', {}, PlugLockPageState> {
    constructor() {
        super('plug_lock/config',
              () => __("plug_lock.script.save_failed"));

        util.addApiEventListener('plug_lock/state', () => {
            this.setState({plug_lock_state: API.get('plug_lock/state')});
        });
    }

    override async sendSave(topic: "plug_lock/config", cfg: PlugLockConfig) {
        // Only when the value actually changed. set_plug_lock_configuration() on the
        // EVSE gates on the *requested* value rather than on a transition, so
        // re-sending the value it already holds can be refused: `true` is rejected
        // while the contactor is closed, and `false` while either Bricklet is still
        // found. An unconditional send would therefore fail Save for no reason - for
        // instance on every save during a charging session.
        if (cfg.enabled === API.get('plug_lock/config').enabled)
            return;

        await API.call('plug_lock/config_update', {enabled: cfg.enabled},
                       () => __("plug_lock.script.save_failed"));
    }

    render(props: {}, s: PlugLockConfig & PlugLockPageState) {
        if (!util.render_allowed())
            return <SubPage name="plug_lock" />;

        const st = s.plug_lock_state;

        // The harness loop, not Bricklet discovery, is what arms the feature. Its state
        // has to be visible: a user who has pulled the link needs to see that it
        // worked, and a user whose loop is broken needs to know that is why charging
        // stopped.
        let loop_note: string = null;

        if (st !== undefined) {
            const found = st.relay_found && st.input_found;
            const verified = st.loop == PlugLockLoop.Verified;
            const challenging = st.loop == PlugLockLoop.Challenging;

            // Reachable only while the feature is enabled - the menu entry is hidden
            // otherwise. That is exactly the case where the user is staring at a
            // blinking error with nothing to act on, so it needs its own message
            // rather than falling through to the broken-loop one.
            if (!found)
                loop_note = __("plug_lock.content.loop_no_bricklets");
            else if (!verified && !challenging)
                loop_note = st.loop == PlugLockLoop.FailStuck ? __("plug_lock.content.loop_stuck")
                                                             : __("plug_lock.content.loop_open");
        }

        // A plug that will not release is stuck in the *safe* state, so it never blocks
        // charging and there is nothing to fix in software. What the user needs is to
        // be told the lock has a mechanical emergency release.
        const stuck = st?.actuation == PlugLockActuation.UnlockFault;

        return <SubPage name="plug_lock" title={__("plug_lock.content.plug_lock")}>
            <SubPage.Config
                id="plug_lock_config_form"
                isDirty={this.isDirty()}
                onSave={this.save}
                onDirtyChange={this.setDirty}>

                {/* Permanent, not conditional on the current value. Switching the plug
                    lock on is a one-way door over the network, and someone who has
                    already done it still needs to be told why the switch will not go
                    back. This warning is the reason the page exists. */}
                <Alert variant="warning">{__("plug_lock.content.one_way")}</Alert>

                <FormRow label={__("plug_lock.content.plug_lock")} help={__("plug_lock.content.plug_lock_help")}>
                    <Switch desc={__("plug_lock.content.plug_lock_desc")}
                            checked={s.enabled}
                            onClick={this.toggle('enabled')}/>
                    {loop_note !== null ? <span class="text-danger">{loop_note}</span> : undefined}
                    {stuck ? <span class="text-danger">{__("plug_lock.content.unlock_fault")}</span> : undefined}
                </FormRow>
            </SubPage.Config>
        </SubPage>;
    }
}

export function pre_init() {
}

export function init() {
}
