import { Calendar, FormatDateString, Options } from './index';
export type RenderState = {
    observer: MutationObserver;
    dirty: boolean;
    fullLayout: boolean;
    options: string;
    context: string;
    structure: string;
    timezone: string;
    month: number;
    count: number;
};
export type Transition = {
    distance: number;
    from: number;
    track: () => void;
    seek: (progress: number) => void;
    settle: (toEnd: boolean) => void;
};
type Component = (self: Calendar) => string;
/** @internal Fixed integration points for the built-in extensions. */
export type MotionExtension = Readonly<{
    name: 'motion';
    collapse: (self: Calendar) => Transition | null;
    bind: (self: Calendar) => void;
    reset: (self: Calendar) => void;
    destroy: (self: Calendar) => void;
    navigate: (self: Calendar, selector: string, route: 'prev' | 'next', render: () => void) => void;
    changeView: (self: Calendar, column: number, render: () => void) => void;
}>;
/** @internal */
export type TimeExtension = Readonly<{
    name: 'time';
    component: Component;
    init: (self: Calendar) => void;
    render: (self: Calendar) => void;
    destroy: (self: Calendar) => void;
}>;
/** @internal */
export type AnnotationsExtension = Readonly<{
    name: 'annotations';
    component: Component;
    tooltip: (self: Calendar, tooltip: HTMLElement | null, date: HTMLElement | null) => void;
    hover: (self: Calendar, target: HTMLElement | null) => void;
    prepare: (self: Calendar, count: number) => ((dates: HTMLElement) => void) | undefined;
    destroy: (self: Calendar) => void;
}>;
/** @internal */
export type WeeksExtension = Readonly<{
    name: 'weeks';
    weekday: (self: Calendar) => ((cell: HTMLElement, title: string, label: string) => void) | undefined;
    component: Component;
    numbersComponent: Component;
    layout: Component;
    init: (self: Calendar) => void;
    dates: (self: Calendar, dates: HTMLElement) => void;
    numbers: (self: Calendar, first: number, days: number, numbers: HTMLElement, dates: HTMLElement) => void;
    date: (self: Calendar, date: HTMLElement, value: FormatDateString) => void;
    shift: (self: Calendar, route: 'prev' | 'next') => void;
    arrows: (self: Calendar, previous: HTMLElement, next: HTMLElement) => void;
    click: (self: Calendar, event: MouseEvent) => void;
    collapseClick: (self: Calendar, event: MouseEvent) => void;
    prepareCollapse: (self: Calendar) => {
        datesEl: HTMLElement;
        rows: HTMLElement[];
        targetRow: HTMLElement;
        wasCollapsed: boolean;
        finish: (toWeek: boolean) => void;
    } | null;
}>;
/** @internal */
export type MonthsExtension = Readonly<{
    name: 'months';
    layout: Component;
    render: (self: Calendar, target?: HTMLElement) => boolean;
    column: (self: Calendar, type: string) => {
        currentValue: number | null;
        columnID: number;
    };
    select: (self: Calendar, type: 'month' | 'year', item: HTMLElement) => void;
    rotate: (self: Calendar, reuse: RenderState) => number[] | undefined;
}>;
type Extensions = {
    motion?: MotionExtension;
    time?: TimeExtension;
    annotations?: AnnotationsExtension;
    weeks?: WeeksExtension;
    months?: MonthsExtension;
};
declare const extensionBrand: unique symbol;
/** An immutable built-in extension. Register it once in the constructor options. */
export type CalendarExtension = Readonly<{
    name: keyof Extensions;
    [extensionBrand]: true;
}>;
export declare const noExtensions: readonly CalendarExtension[];
export declare const getExtensions: (self: Calendar) => Extensions;
export declare const registerExtensions: (self: Calendar) => void;
export declare const validateExtensions: (self: Calendar, options?: Options) => void;
export {};
