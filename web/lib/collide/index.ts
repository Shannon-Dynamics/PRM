/**
 * `collide` — the `Collision` contract and the hand-rolled geometry behind it
 * (Chapter 2). This is the browser's stand-in for the Rust crate's `parry2d`
 * black box: polygon utilities, a `Scene` of walls and obstacles, three
 * checkers (disc, segment chain, rigid footprint), and the ideal range sensor.
 */
export * from './polygon';
export * from './scene';
export * from './collision';
export * from './sensor';
