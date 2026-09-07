# Team612 — GitHub Project Management Guide
### 2026–2027 Season

This guide explains how Team612 uses GitHub to plan, track, and ship work across Programming, Mechanical, Electrical, and Marketing. There are three sections — read the one that matches your role. Everyone should skim the **Golden Rules** first.

---

## Golden Rules (everyone)

1. **No direct pushes to `main`.** All work happens on a personal fork.
2. **Every pull request must reference the ticket (issue) it resolves.** Put `Closes #<issue-number>` in the PR description (or title). This auto-links the PR to the ticket and auto-closes it on merge.
3. **Priority "Urgent" means drop everything, right now.** It is reserved for genuine emergencies — a robot that can't be fixed before an imminent match, a safety issue, etc. Overusing it makes it meaningless for the times it actually matters. Default to High/Medium/Low.
4. We run a **single mono-repo** with **one GitHub Project** (`Team612 - 2026-2027`) that has multiple views: All, Programming, Mechanical, Electrical, Marketing, My Work, and Roadmap.

---

## 1. For Executive Leadership

You don't need to touch code or git commands. Your job is visibility into where the season stands.

**Where to look**
- **Project board → "All" view**: every open ticket across all sub-teams, filterable by status, priority, and milestone.
- **Sub-team views** (Programming / Mechanical / Electrical / Marketing): the same data, scoped to one discipline — useful for checking in with a specific lead.
- **Roadmap view**: a timeline view of tickets. If a ticket has a **Target date** set (available on Feature Request tickets), it plots on the timeline — effectively turning the Roadmap into a Gantt chart of the season. Encourage sub-team leads to set target dates on major features so this stays useful.
- **Milestones** (`1.0`, `1.1`, `1.2`, …): these represent build-season phases/releases. Each milestone page shows a completion percentage and open/closed ticket counts — the fastest way to answer "are we on track."

**What "ticket creation" means for you**
- You generally won't create tickets yourself. If leadership identifies a need (e.g., a new subsystem, a deadline, a marketing deliverable), open a **Feature Request** — it has the right fields (Priority, Start date, Target date, Effort) to make it show up correctly on the Roadmap.
- If something is truly on fire, use the **Urgent** priority — sparingly (see Golden Rules).

**Reading progress at a glance**
- Status column moves left to right: `Triage → Todo → In Progress → Code Review → Test → Done`.
- A large pile in Triage/Todo relative to Done signals a bottleneck worth asking a lead about.
- "Sub-issues progress" on a parent ticket shows how much of a larger effort is complete.

---

## 2. For Mentors / Sub-Team Leads

You own triage, review assignment, and quality control for your sub-team.

**Triage (turning a raw request into an actionable ticket)**
- New issues start in the **Triage** status by default.
- For each new ticket, set:
  - **Priority** (Urgent/High/Medium/Low — hold the line on Urgent).
  - **Labels** (Programming, Mechanical, Electrical, Marketing, plus `duplicate` / `wontfix` as needed).
  - **Milestone** (which build-season phase this belongs to).
  - **Effort**, if the template offers it.
- Move it to **Todo** once it's ready to be picked up, or assign it directly to a student and move it to **In Progress**.

**Reviewing pull requests**
- When a student opens a PR, it should reference its ticket (`Closes #123`). Confirm this before review — a PR without a ticket reference should be sent back.
- **You assign the reviewer** (not the student). Assign yourself or another qualified mentor/senior student.
- **CodeRabbit** runs automatically on every PR as a first-pass AI code reviewer — treat its comments as a starting checklist, not a substitute for your own review.
- Move the ticket to **Code Review** as soon as the PR is open and a reviewer is assigned.
- Once CodeRabbit and the human reviewer both pass it, approve and merge. Move the ticket to **Test** (if it needs on-robot/on-field validation) or straight to **Done**.

**Ongoing upkeep**
- Keep the Labels list clean (currently: `duplicate`, `Electrical`, `Marketing`, `Mechanical`, `Programming`, `wontfix`).
- Open a new **Milestone** at the start of each build-season phase.
- Periodically sweep the Triage column — nothing should sit there for more than a few days.

---

## 3. For Students Working Tickets

This is your step-by-step workflow, every time.

1. **Find your ticket.** Check the **My Work** view in the Project — it automatically filters to tickets assigned to you. Or browse your sub-team's view.
2. **Move it to In Progress** when you start (ask your mentor if it's still in Todo/Triage).
3. **Fork the repo** (one-time setup): click **Fork** on the Team612 repo. This gives you your own copy on GitHub.
4. **Clone your fork** to your computer — not the main Team612 repo.
5. **Create a branch** for your work (don't work directly on `main` in your fork either — good habit for later).
6. **Do the work**, committing as you go with clear commit messages.
7. **Push to your fork.**
8. **Open a pull request** from your fork's branch into `Team612:main`.
   - Title it clearly.
   - In the description, write `Closes #<ticket-number>` — this is **required**. It links your PR to your ticket and will auto-close the ticket when the PR merges.
9. **Move your ticket to Code Review.** Your mentor will assign a reviewer.
10. **Watch for feedback** — both from **CodeRabbit** (automatic AI review, appears as PR comments) and your assigned human reviewer. Push more commits to the same branch to address feedback; they'll show up on the same PR.
11. Once approved and merged, your ticket moves to **Test** or **Done** automatically (or your mentor will move it) — you're done with that ticket!

**Filing new tickets**
- Click **New issue** and pick the right template:
  - **Bug Report** — something is broken. Include steps to reproduce and any logs.
  - **Feature Request** — something new needs to be built.
  - **Enhancement Request** — an existing thing needs to be improved.
- Fill in **Priority** honestly. Don't select **Urgent** unless it truly means "stop what you're doing right now" — ask a mentor if unsure.

---

## Reference: Status & Priority Definitions

| Status | Meaning |
|---|---|
| Triage | New, not yet reviewed by a mentor |
| Todo | Ready to be picked up |
| In Progress | Actively being worked |
| Code Review | PR is open, awaiting review/approval |
| Test | Needs on-robot/on-field validation |
| Done | Complete |

| Priority | Meaning |
|---|---|
| Urgent | Drop everything — true emergencies only |
| High | Important, near-term |
| Medium | Standard priority |
| Low | Nice to have, no rush |
