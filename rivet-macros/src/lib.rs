//! Procedural macros for Rivet RTOS.
//!
//! Provides `#[rivet::task]` for declaring static async tasks that run as
//! real compiler-generated `Future` state machines, stored in a
//! [`rivet::task::TaskCell`] — zero heap allocation, and no nightly
//! features required.

use proc_macro::TokenStream;
use quote::{format_ident, quote};
use syn::{parse_macro_input, ItemFn, LitInt};

/// Declare the application entry point. Expands to the `#[no_mangle]
/// extern "C" fn rivet_main() -> !` that `rivet-rt`'s boot code (`_start`
/// on RISC-V, `Reset` on Cortex-M) calls after bss/data init, with
/// [`rivet::init`](https://docs.rs/rivet) inserted automatically before
/// the function body runs.
///
/// ```ignore
/// #[rivet::main]
/// fn main() -> ! {
///     rivet::println!("hello");
///     rivet::spawn_ptask!(stack = 512, priority = 1, entry = my_task, arg = ());
///     rivet::run()
/// }
/// ```
///
/// Takes no arguments; the annotated function must take no parameters
/// (link the board/arch you want via `use rivet_bsp_...  as _;` instead —
/// see `docs/porting.md`).
#[proc_macro_attribute]
pub fn main(attr: TokenStream, item: TokenStream) -> TokenStream {
    if !attr.is_empty() {
        return syn::Error::new_spanned(
            proc_macro2::TokenStream::from(attr),
            "#[rivet::main] takes no arguments",
        )
        .to_compile_error()
        .into();
    }
    let input = parse_macro_input!(item as ItemFn);
    if !input.sig.inputs.is_empty() {
        return syn::Error::new_spanned(
            &input.sig.inputs,
            "#[rivet::main] functions take no parameters",
        )
        .to_compile_error()
        .into();
    }
    let fn_attrs = &input.attrs;
    let fn_block = &input.block;

    let expanded = quote! {
        #[no_mangle]
        #(#fn_attrs)*
        extern "C" fn rivet_main() -> ! {
            ::rivet::init();
            #fn_block
        }
    };

    TokenStream::from(expanded)
}

/// Declare a static async task.
///
/// The annotated function must be `async fn name() { ... }` — no
/// parameters (shared state goes through `static`s, matching the usual
/// embedded pattern for peripherals/queues). The body may freely use
/// `.await` — `Sleep`, `Semaphore::acquire()`, `Channel::send()/recv()`
/// all work as real futures polled by the executor.
///
/// # Attributes
/// - `priority` (required): Task priority (0 = lowest, 31 = highest).
/// - `stack` (optional): bytes reserved for the future's state machine
///   (default 512). Increase this if you get a
///   "task future exceeds reserved stack size" panic at boot.
///
/// # Example
/// ```ignore
/// #[rivet::task(priority = 1, stack = 256)]
/// async fn blinky() {
///     loop {
///         rivet::time::Sleep::<500_000>::new().await; // 500ms
///         toggle_led();
///     }
/// }
/// ```
///
/// # How it works
///
/// `F` (the compiler-generated `Future` type of an `async fn`) is
/// unnameable on stable Rust, so it can't appear in a `static`'s type.
/// Instead the macro declares `static CELL: TaskCell<STACK_SIZE>`
/// (`STACK_SIZE` is just a `usize`, always nameable) and generates a
/// thin non-generic wrapper function that calls the crate's generic
/// `TaskCell::poll::<F>`, letting the compiler monomorphize the actual
/// future read/write per task without ever needing to write `F` down.
#[proc_macro_attribute]
pub fn task(attr: TokenStream, item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as ItemFn);
    let fn_name = &input.sig.ident;
    let fn_visibility = &input.vis;
    let fn_sig = &input.sig;
    let fn_attrs = &input.attrs;
    let task_body = &input.block;

    if input.sig.asyncness.is_none() {
        return syn::Error::new_spanned(&input.sig, "#[rivet::task] requires an `async fn`")
            .to_compile_error()
            .into();
    }
    if !input.sig.inputs.is_empty() {
        return syn::Error::new_spanned(
            &input.sig.inputs,
            "#[rivet::task] functions currently take no parameters; \
             use a `static` for shared state (peripherals, queues, etc.)",
        )
        .to_compile_error()
        .into();
    }

    let mut priority: u8 = 0;
    let mut stack_size: usize = 512;
    let mut saw_priority = false;

    let parser = syn::meta::parser(|meta| {
        if meta.path.is_ident("priority") {
            let value = meta.value()?;
            let lit: LitInt = value.parse()?;
            priority = lit.base10_parse::<u8>()?;
            saw_priority = true;
        } else if meta.path.is_ident("stack") {
            let value = meta.value()?;
            let lit: LitInt = value.parse()?;
            stack_size = lit.base10_parse::<usize>()?;
        } else {
            return Err(
                meta.error("unsupported #[rivet::task] attribute; expected `priority` or `stack`")
            );
        }
        Ok(())
    });
    // `parse_macro_input!` with a parser that returns `()` both parses the
    // attribute and converts parse errors into compile errors.
    parse_macro_input!(attr with parser);

    if !saw_priority {
        return syn::Error::new_spanned(
            fn_name,
            "#[rivet::task] requires `priority = N`, e.g. #[rivet::task(priority = 1)]",
        )
        .to_compile_error()
        .into();
    }

    let poll_fn_name = format_ident!("__rivet_poll_{}", fn_name);
    let completed_fn_name = format_ident!("__rivet_completed_{}", fn_name);
    let cell_name = format_ident!("__RIVET_CELL_{}", fn_name);
    let reg_name = format_ident!("__RIVET_REG_{}", fn_name);

    let expanded = quote! {
        // The user's async fn, unchanged — the compiler generates its
        // Future state machine as normal.
        #(#fn_attrs)*
        #fn_visibility #fn_sig #task_body

        // Zero-alloc storage for the future, sized (not typed) generically.
        #[allow(non_upper_case_globals)]
        static #cell_name: ::rivet::task::TaskCell<#stack_size> = ::rivet::task::TaskCell::new();

        // Thin non-generic wrapper: calls the generic, monomorphized
        // TaskCell::poll::<F> where F is inferred from `#fn_name`.
        #[allow(non_snake_case)]
        unsafe fn #poll_fn_name(
            _user_data: *mut (),
            waker: &::core::task::Waker,
        ) -> ::core::task::Poll<()> {
            #cell_name.poll(#fn_name, waker)
        }

        // Type-erased completed probe for this concrete cell.
        #[allow(non_snake_case)]
        unsafe fn #completed_fn_name(_user_data: *mut ()) -> bool {
            #cell_name.is_completed()
        }

        // Registration entry discovered by the executor at boot.
        #[link_section = ".rivet_tasks"]
        #[used]
        #[allow(non_upper_case_globals)]
        static #reg_name: ::rivet::task::TaskReg = ::rivet::task::TaskReg {
            priority: #priority,
            index_in_priority: 0,
            _reserved: [0; 2],
            poll_fn: #poll_fn_name as unsafe fn(*mut (), &::core::task::Waker) -> ::core::task::Poll<()>,
            completed_fn: #completed_fn_name as unsafe fn(*mut ()) -> bool,
            user_data: ::core::ptr::null_mut(),
        };
    };

    TokenStream::from(expanded)
}
