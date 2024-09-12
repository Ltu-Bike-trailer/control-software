use proc_macro::TokenStream;
use quote::quote;
use syn::{bracketed, parse::Parse, parse_macro_input, Ident, LitStr, Token};

fn _hash(data: &str) -> u32 {
    let mut prev_sum: u32 = 0;
    data.as_bytes()
        .iter()
        .map(|num| {
            prev_sum = prev_sum ^ *num as u32;
            prev_sum
        })
        .sum::<u32>()
}

#[proc_macro]
pub fn hash(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as LitStr).value();

    let hash = _hash(&input);

    println!("Hash {hash}");

    quote! {
        #hash
    }
    .into()
}

#[proc_macro]
pub fn string_to_bytes(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as LitStr).value();
    let bytes = input.as_bytes();

    let len = (bytes.len() >> 1).to_le_bytes();

    quote! {
        [#(#len,)*#(#bytes,)*]
    }
    .into()
}

struct IdentArray(Vec<Ident>);

impl Parse for IdentArray {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let content;
        bracketed!(content in input);
        let mut idents = Vec::new();
        while !content.is_empty() {
            idents.push(content.parse()?);
            let _: Token![,] = content.parse()?;
        }
        Ok(Self(idents))
    }
}

#[proc_macro]
pub fn numel(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as IdentArray).0.len();
    quote! {
        #input
    }
    .into()
}
