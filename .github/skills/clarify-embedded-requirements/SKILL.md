---
name: clarify-embedded-requirements
description: Convert vague, incomplete, or multi-part CurRobo firmware prompts into evidence-backed requirements, explicit assumptions, acceptance criteria, interfaces, constraints, risks, and an ordered implementation plan. Use before complex features, architecture-affecting changes, hardware integration, control behavior, protocols, refactors, or whenever the user's wording admits materially different implementations.
---

# Clarify embedded requirements

## Understand intent before solution

Extract four layers without silently filling gaps:

1. **Outcome** - what observable user or robot behavior must change.
2. **Constraints** - hardware, architecture, interfaces, timing, memory, safety,
   compatibility, tooling, and allowed file scope.
3. **Evidence** - repository facts, `.ioc`, existing behavior, logs,
   measurements, and user-provided hardware information.
4. **Unknowns** - facts that cannot be safely inferred.

Separate explicit requirements, repository-derived facts, reasonable defaults,
and unresolved decisions. Never restate a proposed implementation as though it
were the user's requirement.

## Apply proportional clarification

Classify the request:

- **Small/reversible**: one local implementation with no hardware, public API,
  timing, safety, persistence, or protocol ambiguity. Proceed with stated
  assumptions.
- **Medium**: multiple modules or meaningful timing/resource effects. Present a
  compact contract and plan, then proceed unless a decision is blocking.
- **High-risk/irreversible**: pin/clock/MPU/linker changes, actuator safety,
  persistent calibration, protocol compatibility, watchdog policy, public API,
  or architecture. Stop for explicit confirmation of unresolved decisions.

Do not ask questions already answered by the repository. Ask the smallest
number of decision questions that materially change the result. For each,
explain the consequence of the alternatives and recommend a default when safe.

## Produce a requirement contract

For nontrivial work, state:

- Goal and non-goals
- Inputs, outputs, units, ranges, coordinate/sign conventions
- Invocation context and lifecycle
- Nominal rate, deadline, jitter tolerance, timeout
- State transitions and initialization/reset behavior
- Error detection, degraded mode, actuator-safe state, recovery
- Data ownership and ISR/task concurrency
- Hardware/peripheral and protocol constraints
- Backward-compatibility requirements
- Quantified acceptance criteria
- Assumptions and decisions still requiring confirmation

Acceptance criteria must be observable. Prefer forms such as:

`Given <initial state>, when <stimulus>, then <observable result> within
<limit>, while <safety/resource constraint>.`

Avoid unverifiable criteria such as "works correctly," "stable," or "fast."

## Decompose into a task graph

Create tasks that are independently reviewable and leave the project buildable:

1. Repository reconnaissance and baseline reproduction
2. Contract/interface or data-model change
3. Lowest hardware/BSP capability
4. Reusable module/protocol behavior
5. Application/service integration
6. Failure handling, observability, and safety
7. Host/static/build verification
8. Board/HIL verification
9. Documentation or protocol update when required
10. Git diff review and integration preparation

For each task state:

- Deliverable and affected layer
- Dependencies and files likely involved
- Verification evidence
- Risk and rollback point
- Definition of done

Split tasks at behavior or interface boundaries, not arbitrary file counts.
Identify the critical path and tasks that can be done independently. Do not
create placeholder scaffolding that cannot be compiled or reviewed.

## Handoff

End with one of:

- **Ready** - contract is sufficient; proceed with the first task.
- **Ready with assumptions** - list reversible assumptions and proceed.
- **Blocked by decision** - ask only the blocking decision questions.

Keep the output compact unless the user explicitly requests a full design
document.
